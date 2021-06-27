#include "pendulum_utils/util.hpp"
#include <fstream>
#include <iostream>

using namespace std;
namespace pendulum
{
namespace utils
{
HistReport::HistReport(int bins_num, uint64_t bin_width_ns, int max_topn) {
  bins_num_ = bins_num;
  bins_.resize(bins_num_);
  bin_width_ns_ = bin_width_ns;

  max_topn_ = max_topn;
  top_ns_.resize(max_topn_);
  top_idx_.resize(max_topn_);
  idx_ = 0;
  head_idx_ = 0;
}

void HistReport::add(uint64_t ns) {
  int bi = ns / bin_width_ns_;
  if ( bi < bins_num_) {
    bins_[bi]++;
  }

  // update topn. index 0-max_topn_ corresponds to top1-topn.
  for (int i=0 ; i<max_topn_ ; i++) {
    if (ns < top_ns_[i]) {
      continue;
    }
    // shift backward from i index.
    for (int j=max_topn_-1 ; j>i ; j--) {
      top_ns_[j] = top_ns_[j-1];
      top_idx_[j] = top_idx_[j-1];
    }
    // insert new data to i index;
    top_ns_[i] = ns;
    top_idx_[i] = idx_;
    break;
  }
  idx_++;
}

void HistReport::histToCsv(std::string filename) {
  ofstream hist_ofs(filename);
  if (!hist_ofs) {
    cerr << "failed to open " << filename << "." << endl;
    return;
  }
  hist_ofs << "latency,count" << endl;
  for (int i = 0; i < bins_num_; i++) {
    hist_ofs << i << "," << bins_[i] << endl;
  }
}

void HistReport::topnToHist(std::string filename) {
  ofstream topn_ofs(filename);
  if (!topn_ofs) {
    cerr << "failed to open " << filename << "." << endl;
    return;
  }
  topn_ofs << "latency,index" << endl;
  for (int i = 0; i < max_topn_; i++) {
    topn_ofs << top_ns_[i] << "," << top_idx_[i] << endl;
  }
}

TimeSeriesReport::TimeSeriesReport(int max_data_num) {
  max_data_num_ = max_data_num;
  data_.resize(max_data_num_);
  idx_ = 0;
}
void TimeSeriesReport::add(uint64_t ns) {
  if (idx_ < max_data_num_) {
    data_[idx_] = ns;
  }
  idx_++;
}

void TimeSeriesReport::toCsv(std::string filename) {
  ofstream ofs(filename);

  if (!ofs) {
    cerr << "failed to open " << filename << "." << endl;
    return;
  }

  ofs << "idx,latency" << endl;
  for (int i = 0; i < max_data_num_; i++) {
    ofs << i << "," << data_[i] << endl;
  }
}

std::chrono::nanoseconds toChronoDuration(timespec ts) {
  using namespace std::chrono;
  auto duration = seconds{ts.tv_sec} + nanoseconds{ts.tv_nsec};
  return duration_cast<nanoseconds>(duration);
}

}  // namespace utils
}  // namespace pendulum