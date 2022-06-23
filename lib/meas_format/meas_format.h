#ifndef MEAS_FORMAT_H
#define MEAS_FORMAT_H
#include <functional>
#include <ostream>
#include <sstream>
#include <vector>
#ifdef ROWFMT_BENCHMARK
#include <Arduino.h>
#endif
class Column {
 public:
  Column(std::function<void(std::ostream &)> header,
         std::function<void(std::ostream &)> row)
      : header(header), row(row){};
  Column(const char *hdr, std::function<void(std::ostream &)> row) : row(row) {
    header = [hdr](std::ostream &ss) { ss << hdr; };
  }

 public:
  std::function<void(std::ostream &)> header;
  std::function<void(std::ostream &)> row;
};

class Rowformatter {
 public:
  Rowformatter(){};
  Rowformatter(std::vector<Column> cols) : cols(cols){};
  void header(std::ostream &s, const char *delim = ", ") {
    for (auto &col : cols) {
      col.header(s);
      s << delim;
    }
    s << "\n";
  }
  std::string header_str() {
    std::stringstream ss{};
    this->header(ss);
    return ss.str();
  }
  void row(std::ostream &s, const char *delim = ", ") {
    for (auto &col : cols) {
#ifdef ROWFMT_BENCHMARK
      auto starttime = millis();
#endif
      col.row(s);
#ifdef ROWFMT_BENCHMARK
      s << " took " << (millis() - starttime) << " ms.";
#endif
      s << delim;
    }
    s << "\n";
  }
  std::string row_str() {
    std::stringstream ss{};
    this->row(ss);
    return ss.str();
  }

 private:
  std::vector<Column> cols;
};

#endif  // MEAS_FORMAT_H
