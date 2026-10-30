#include "lidar_state_recorder.h"
#include "rapidjson/document.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sched.h>
#include <unistd.h>

using namespace livox_ros::raw_state;

namespace {
void Require(bool value, const char* reason) {
  if (!value) { std::cerr << "FAIL: " << reason << '\n'; std::exit(1); }
}
std::vector<rapidjson::Document> Read(const std::string& path) {
  std::ifstream stream(path);
  std::vector<rapidjson::Document> rows;
  std::string line;
  while (std::getline(stream, line)) {
    rapidjson::Document row;
    row.Parse(line.c_str());
    Require(!row.HasParseError() && row.IsObject(), "each completed line is valid JSON");
    rows.emplace_back(std::move(row));
  }
  return rows;
}
Recorder::Options Options(const std::string& directory) {
  Recorder::Options options;
  options.directory = directory; options.role = "right";
  options.handle = 60145088; options.ip = "192.168.1.3";
  return options;
}
const char* kFullJson = R"({"cur_work_state":1,"hms_code":[4294967295,2,1,0,0,0,0,0],"time_sync_type":1,"local_time_now":18446744073709551615,"last_sync_time":18446744073709551614,"time_offset":-5000,"core_temp":-12345,"powerup_cnt":42,"sn":"1234567890123456","version_app":[1,2,3,4],"version_loader":[255,0,2,3],"version_hardware":[4,3,2,1]})";

void TestParsing() {
  Snapshot s;
  Require(ParseSdkJson(kFullJson, std::strlen(kFullJson), s), "valid SDK JSON");
  Require(s.status[Hms] == FieldStatus::Valid && s.hms[0] == UINT32_MAX && s.hms[1] == 2 && s.hms[2] == 1,
      "all eight raw HMS codes and array order preserved");
  Require(s.local_time == UINT64_MAX && s.time_offset == -5000 && s.core_temp == -12345 && s.powerup_count == 42,
      "full precision raw timestamps, signed offset/temperature and power-up counter");
  Require(s.app_version[3] == 4 && s.loader_version[0] == 255 && s.serial[16] == '\0', "bounded versions and serial");
  Require(s.status[Voltage] == FieldStatus::NotAvailable && s.status[Uptime] == FieldStatus::NotAvailable,
      "unsupported voltage/uptime not invented");
  const char* invalid = R"({"hms_code":[1],"cur_work_state":256,"core_temp":1.5,"sn":"abcdefghijklmnopq","time_offset":18446744073709551615,"version_app":[0,1,2,256]})";
  Require(ParseSdkJson(invalid, std::strlen(invalid), s), "malformed fields do not invalidate other JSON fields");
  for (auto field : {Hms, WorkState, CoreTemp, Serial, TimeOffset, AppVersion})
    Require(s.status[field] == FieldStatus::Invalid, "present invalid values distinguished from missing");
  Require(s.status[LocalTime] == FieldStatus::Missing, "missing local_time is not zero");
  Require(!ParseSdkJson("{", 1, s) && !ParseSdkJson(nullptr, 0, s) && !ParseSdkJson("{}", 16385, s),
      "invalid/truncated/oversize JSON rejected before reads");
  Require(ParseSdkJson("{}", 2, s) && s.status[Hms] == FieldStatus::Missing, "empty state not HMS clear");
  const std::string invalid_serial = std::string("{\"sn\":\"") + static_cast<char>(0xff) + "\",\"cur_work_state\":4}";
  Require(ParseSdkJson(invalid_serial.data(), invalid_serial.size(), s) && s.status[Serial] == FieldStatus::Invalid &&
      s.status[WorkState] == FieldStatus::Valid && s.work_state == 4, "invalid SN byte cannot corrupt other valid fields");
  const auto query = Snapshot::QueryFields();
  Require(query.status[CoreTemp] == FieldStatus::NoResponseLength && query.status[Serial] == FieldStatus::NoResponseLength &&
      query.status[Hms] == FieldStatus::Missing, "query does not extend lengthless TLV reads or fill older push values");
}

void TestQueue() {
  RecordQueue queue(2);
  Record row; row.wall_ns = 1000; row.steady_ns = 1000;
  Require(queue.Push(row), "queue accepts first");
  row.wall_ns = 500; row.steady_ns = 2000;
  Require(queue.Push(row), "wall time may move backwards");
  Require(!queue.Push(row) && queue.dropped() == 1, "full bounded queue counts drop without overwrite");
  Record out;
  Require(queue.Pop(out) && out.seq == 1 && out.wall_ns == 1000, "first accepted sequence");
  Require(queue.Pop(out) && out.seq == 2 && out.wall_ns == 500 && out.steady_ns == 2000, "steady/sequence order survives wall-clock rollback");
  Require(!queue.Pop(out), "queue empty");
  Require(queue.Push(row) && queue.Pop(out) && out.seq == 3, "drops do not consume sequence");

  RecordQueue push(8);
  row.source = Source::StatePush; row.steady_ns = 1000000000;
  row.fields.status[WorkState] = FieldStatus::Valid; row.fields.work_state = 1;
  Require(push.Push(row), "first push");
  row.steady_ns += 1000000;
  Require(!push.Push(row) && push.suppressed() == 1 && push.dropped() == 0, "only identical state suppresses without gap");
  row.fields.work_state = 2;
  Require(!push.Push(row) && push.dropped() == 1, "changed push rate limiting counted as dropped evidence");
  row.steady_ns += 200000000;
  Require(push.Push(row), "changed push accepted after rate limit");
  row.source = Source::Query;
  Require(push.Push(row), "periodic query never suppressed by push deduplication/rate limit");

  RecordQueue concurrent(8);
  std::atomic<int> remaining{4};
  std::vector<std::thread> producers;
  for (int i = 0; i < 4; ++i) producers.emplace_back([&]() {
    for (int j = 0; j < 1000; ++j) concurrent.Push(Record{});
    --remaining;
  });
  uint64_t consumed = 0;
  while (true) {
    if (concurrent.Pop(out)) Require(out.seq == ++consumed, "concurrent producers preserve accepted FIFO sequence");
    else if (!remaining) break;
    else std::this_thread::yield();
  }
  for (auto& thread : producers) thread.join();
  while (concurrent.Pop(out)) Require(out.seq == ++consumed, "final concurrent queue flush order");
  Require(consumed + concurrent.dropped() == 4000, "all concurrent attempts either accepted or counted dropped");
  concurrent.Close();
  Require(!concurrent.Push(Record{}), "closed queue never reports late submission accepted");
}

void TestConcurrentStop(const std::string& root) {
  auto options = Options(root + "/concurrent_stop"); options.queue_capacity = 1024;
  Recorder recorder(options);
  std::atomic<size_t> accepted{0}, attempted{0};
  std::thread producer([&]() {
    for (int i = 0; i < 1000; ++i) {
      if (recorder.RecordObservation(Record{})) ++accepted;
      ++attempted;
    }
  });
  while (attempted < 20) std::this_thread::yield();
  recorder.Stop(); producer.join();
  const auto rows = Read(options.directory + "/right_state.jsonl");
  Require(rows.size() == accepted + 2, "concurrent close flushes every accepted row and rejects late producers");
}

void TestScenarioFixture(const std::string& root) {
  auto options = Options(root + "/scenario_native");
  Recorder recorder(options);
  Record row; row.source = Source::StatePush;
  ParseSdkJson(kFullJson, std::strlen(kFullJson), row.fields);
  row.fields.hms.fill(0); row.fields.local_time = 10000000000ULL; row.fields.last_sync = 9900000000ULL;
  row.fields.core_temp = 4200;  // raw, deliberately not converted to a temperature unit
  row.steady_ns = 1000000000LL; row.wall_ns = 1793347200000000000LL;
  std::strcpy(row.detail.data(), "synthetic_timeline_fixture");
  for (int state = 0; state < 4; ++state) {
    row.fields.sync_type = state == 1 ? 0 : 1;
    row.fields.work_state = state == 2 ? 4 : 1;
    row.fields.hms.fill(0);
    if (state == 2) row.fields.hms = {{0x02160003, 0x02100003, 0x04050002, 0, 0, 0, 0, 0}};
    bool accepted = false;
    for (int retry = 0; retry < 100 && !accepted; ++retry) {
      accepted = recorder.RecordObservation(row);
      if (!accepted) std::this_thread::yield();
    }
    Require(accepted, "scenario state accepted");
    row.steady_ns += 300000000; row.wall_ns += 300000000;
    row.fields.local_time += 300000000; row.fields.last_sync += 300000000;
  }
  recorder.Stop();
  // Keep the actual C++ writer's observation lines verbatim. The fixture uses a
  // logical clock; real lifecycle timestamps are omitted from this test-only run.
  ::mkdir((root + "/scenario").c_str(), 0750);
  ::mkdir((root + "/scenario/diagnostics").c_str(), 0750);
  std::ifstream source(options.directory + "/right_state.jsonl");
  std::ofstream dest(root + "/scenario/diagnostics/right_state.jsonl");
  std::string line; size_t count = 0;
  while (std::getline(source, line)) {
    rapidjson::Document doc; doc.Parse(line.c_str());
    if (std::string(doc["source"].GetString()) == "state_push") { dest << line << '\n'; ++count; }
  }
  Require(count == 4, "Normal -> PTP change -> work4 with three HMS codes -> Normal fixture retained");
}

void TestWriteAndFlush(const std::string& root) {
  const int main_nice = ::getpriority(PRIO_PROCESS, 0);
  cpu_set_t main_cpus_before, main_cpus_after;
  Require(::sched_getaffinity(0, sizeof(main_cpus_before), &main_cpus_before) == 0, "read caller CPU affinity");
  auto options = Options(root + "/flush");
  options.queue_capacity = 1024;
  Recorder recorder(options);
  Record full; full.source = Source::StatePush;
  Require(ParseSdkJson(kFullJson, std::strlen(kFullJson), full.fields), "full fixture parsed");
  bool full_accepted = false;
  for (int i = 0; i < 100 && !full_accepted; ++i) {
    full_accepted = recorder.RecordObservation(full);
    if (!full_accepted) std::this_thread::yield();
  }
  Require(full_accepted, "full fixture accepted despite allowable producer contention");
  Record row; row.result = QueryResult::Success; row.fields = Snapshot::QueryFields();
  row.fields.status[WorkState] = FieldStatus::Valid; row.fields.work_state = 1;
  size_t accepted = 0;
  for (int i = 0; i < 100; ++i) if (recorder.RecordObservation(row)) ++accepted;
  recorder.RecordLifecycle(Source::Error, "test_marker");
  recorder.Stop();
  Require(::getpriority(PRIO_PROCESS, 0) == main_nice, "writer nice adjustment does not change caller priority");
  Require(::sched_getaffinity(0, sizeof(main_cpus_after), &main_cpus_after) == 0 && CPU_EQUAL(&main_cpus_before, &main_cpus_after),
      "writer pinning does not change caller affinity");
  const auto rows = Read(options.directory + "/right_state.jsonl");
  Require(rows.size() >= accepted + 2, "normal shutdown flushes all accepted observations plus lifecycle");
  Require(std::string(rows.front()["source"].GetString()) == "startup" &&
      std::string(rows.back()["source"].GetString()) == "shutdown", "startup and final shutdown emitted");
  uint64_t seq = 0; int64_t steady = 0; bool saw_full = false, saw_query = false;
  for (const auto& record : rows) {
    Require(record["seq"].GetUint64() == ++seq && record["steady_ns"].GetInt64() >= steady, "writer retains accepted FIFO monotonic order");
    steady = record["steady_ns"].GetInt64();
    Require(record["schema"].GetInt() == 1 && std::string(record["role"].GetString()) == "right", "schema role metadata");
    Require(std::string(record["writer_scheduling"].GetString()) == "nice10_nonisolated", "writer verifies low-priority nonisolated scheduling before IO");
    Require(record["fields"]["voltage"].IsNull() && record["fields"]["uptime"].IsNull(), "unsupported values are null in JSON");
    if (std::string(record["source"].GetString()) == "state_push") {
      saw_full = true;
      Require(record["fields"]["local_time_now"].GetUint64() == UINT64_MAX, "uint64 survives JSON serialization");
      Require(std::string(record["field_sources"]["core_temp"].GetString()) == "state_push", "per-field source retained");
      Require(std::string(record["core_temp_unit"].GetString()) == "sdk_raw_unit_unspecified", "no temperature scale invented");
    }
    if (std::string(record["source"].GetString()) == "query") {
      saw_query = true;
      Require(record["fields"]["core_temp"].IsNull() && std::string(record["field_status"]["core_temp"].GetString()) == "no_response_length",
          "query never reuses older push temperature");
    }
  }
  Require(saw_full && saw_query, "both callback sources persisted");
  Require(recorder.counters().write_errors == 0, "healthy disk has no write errors");
}

void TestRotation(const std::string& root) {
  auto options = Options(root + "/rotate"); options.file_bytes = 4096; options.queue_capacity = 256;
  Recorder recorder(options);
  Record row;
  for (int i = 0; i < 80; ++i) recorder.RecordObservation(row);
  recorder.Stop();
  uint64_t previous = 0; size_t total_size = 0;
  for (int suffix = 3; suffix >= 0; --suffix) {
    const auto path = options.directory + "/right_state.jsonl" + (suffix ? "." + std::to_string(suffix) : "");
    struct stat st{};
    Require(::stat(path.c_str(), &st) == 0 && st.st_size > 0 && st.st_size <= 4096, "four files each respect limit");
    total_size += st.st_size;
    for (const auto& row : Read(path)) {
      Require(row["seq"].GetUint64() > previous, ".3 oldest through current preserve accepted order");
      previous = row["seq"].GetUint64();
    }
  }
  Require(total_size <= 4 * options.file_bytes, "per-role total bounded by four files");
  Require(::access((options.directory + "/right_state.jsonl.4").c_str(), F_OK) != 0, "no fifth rotation");
  const auto rows = Read(options.directory + "/right_state.jsonl");
  Require(std::string(rows.back()["source"].GetString()) == "shutdown", "latest final lifecycle survives rotation");
  // Simulate SIGKILL leaving the last line partial, then a fresh process session.
  { std::ofstream append(options.directory + "/right_state.jsonl", std::ios::app); append << "{\"partial\":"; }
  Recorder next(options); next.Stop();
  const auto next_rows = Read(options.directory + "/right_state.jsonl");
  Require(next_rows.size() == 2 && std::string(next_rows.front()["session"].GetString()) != rows.back()["session"].GetString(),
      "new session rotates partial final line instead of joining JSON records");
}

void TestWriteFailure(const std::string& root) {
  const auto blocked = root + "/blocked";
  { std::ofstream file(blocked); file << "not a directory"; }
  auto options = Options(blocked);
  Recorder recorder(options);
  const auto begin = std::chrono::steady_clock::now();
  Record row;
  for (int i = 0; i < 10000; ++i) recorder.RecordObservation(row);
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  Require(elapsed < std::chrono::seconds(1), "callbacks do not wait for failed file IO");
  for (int i = 0; i < 100 && recorder.counters().write_errors == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  Require(recorder.counters().write_errors > 0, "write failures counted");
  Require(::unlink(blocked.c_str()) == 0, "remove temporary fixture blocker");
  recorder.RecordLifecycle(Source::Error, "after_write_recovery");
  recorder.Stop();
  const auto rows = Read(blocked + "/right_state.jsonl");
  Require(!rows.empty() && rows.back()["write_errors_total"].GetUint64() > 0, "later writable row reports preceding write errors");
  Require(rows.back()["dropped_total"].GetUint64() > 0, "bounded failure load reports producer loss");
  Recorder disabled(Options(""));
  Require(!disabled.enabled() && !disabled.RecordObservation(row), "unset directory is fully disabled");
}
}  // namespace

int main(int argc, char** argv) {
  char temp[] = "/tmp/livox-state-recorder-XXXXXX";
  const char* generated = ::mkdtemp(temp);
  Require(generated != nullptr, "temporary directory");
  const std::string root = argc > 1 ? argv[1] : generated;
  ::mkdir(root.c_str(), 0750);
  TestParsing(); TestQueue(); TestWriteAndFlush(root); TestRotation(root); TestWriteFailure(root);
  TestConcurrentStop(root); TestScenarioFixture(root);
  std::cout << "PASS: parse/missing, bounded queue/drop/rate, ordering, rotation, flush, write failure/recovery\n";
  std::cout << "JSONL fixture: " << root << "/flush/right_state.jsonl\n";
}
