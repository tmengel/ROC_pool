#ifndef __H2GCROC3_10G_POOL_H__
#define __H2GCROC3_10G_POOL_H__

#include "ROC_pool.h"
#include <map>

// Pool for packet format 302 (IDH2GCROC3_10G), same API as
// oncsSub_idh2gcroc3_10G. A "sample" holds the values of all channels
// for one timestamp; a waveform is a run of samples less than
// CONTIGUOUS_CLOCK apart. The lines of neighbouring samples can arrive
// interleaved (a link runs a sample ahead now and then), so samples are
// kept open, keyed by timestamp, until the stream is REORDER_WINDOW clocks
// past them, and closed in timestamp order.
//
// "NR_WF" is the number of complete waveforms on offer, wf = 0 .. NR_WF-1.
// The sample-indexed calls ("SAMPLES", "TIMESTAMP", "CMN", "CALIB",
// iValue(sample, ch [, what])) count the samples of the waveforms on offer.

class h2gcroc3_10G_pool : public ROC_pool
{
public:
  h2gcroc3_10G_pool(const unsigned int low_mark = 200, const unsigned int high_mark = 600);
  ~h2gcroc3_10G_pool() {}

  int    iValue(const int sample, const int ch);
  int    iValue(const int sample, const int ch, const char *what);
  int    iValue(const int n, const char *what);
  int    iValue(const int wf, const int sample, const int ch);
  int    iValue(const int wf, const int sample, const int ch, const char *what);
  long long lValue(const int wf, const int sample, const int ch);

  void dump(std::ostream &os = std::cout);

  static const unsigned int MAX_CHIPS = 4;
  static const uint64_t CONTIGUOUS_CLOCK = 164;
  static const uint64_t REORDER_WINDOW = 4*CONTIGUOUS_CLOCK;  // a sample stays open this long behind the newest timestamp
  static const uint64_t RESET_CLOCK = 100000000;              // a timestamp this far back is a counter reset

  // problems counted over the whole stream (the per-Packet decoder counts per Packet)
  unsigned long long crc_errors() const { return _crc_errors; }
  unsigned long long hamming_errors() const { return _hamming_errors; }
  unsigned long long bad_lines() const { return _bad_lines; }
  unsigned long long late_lines() const { return _late_lines; }
  unsigned long long resets() const { return _resets; }

protected:

  int decode_datagram(const uint32_t *d, const unsigned int nbytes);
  void close_open();
  void update_offer();
  void drop_offer();
  bool has_pending() const;

  int decode_line(const uint8_t *d);

  static unsigned int u4frombyte(const uint8_t *v);
  static uint64_t u8frombyte(const uint8_t *v);
  static unsigned int crc32_frame(const uint8_t *d, const int n);

  struct sample
  {
    unsigned int ADC[72*MAX_CHIPS];
    unsigned int tot[72*MAX_CHIPS];
    unsigned int toa[72*MAX_CHIPS];
    unsigned int error[2*MAX_CHIPS];  // H1 H2 H3 hamming error bits per half-chip
    unsigned int trigger_in;
    unsigned int trigger_out;
    unsigned int event_counter;
    uint64_t timestamp;
    unsigned int cmn;
    unsigned int calib;
    unsigned int crc32;
    unsigned int crc_error[2*MAX_CHIPS];   // per link (2*chip + group): 1 if the line's CRC-32 did not match
  };

  void close_sample(const sample &s);           // append a closed sample to the waveforms
  void close_samples(const uint64_t before);    // close the open samples below this timestamp, in order

  // samples are numbered from the start of the stream; _samples[0] is
  // sample number _sample_base
  struct wf_bounds
  {
    unsigned long long first;
    unsigned int length;
  };

  const sample *getSample(const int wf, const int s) const;
  const sample *getSample(const int s) const;

  std::deque<sample> _samples;      // closed samples
  unsigned long long _sample_base;

  std::deque<wf_bounds> _waveforms; // closed waveforms
  bool _have_open_wf;
  wf_bounds _open_wf;
  uint64_t _last_closed_ts;

  std::map<uint64_t, sample> _open; // samples still receiving lines, by timestamp
  uint64_t _max_timestamp;
  uint64_t _closed_before;          // samples below this are closed; a line for one is late

  unsigned long long _crc_errors;
  unsigned long long _hamming_errors;
  unsigned long long _bad_lines;    // no line marker, or chip / half out of range
  unsigned long long _late_lines;   // arrived after their sample was closed (dropped)
  unsigned long long _resets;

  unsigned int _nr_offered;
  unsigned long long _nr_offered_samples;
  int _nr_channels;
};

#endif
