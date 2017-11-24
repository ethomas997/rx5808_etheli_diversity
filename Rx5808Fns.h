// Rx5808Fns.h

#ifndef RX5808FNS_H_
#define RX5808FNS_H_

uint8_t getChannelSortTableIndex(uint8_t channelIndex);
uint8_t getChannelSortTableEntry(int idx);
uint16_t getChannelFreqTableEntry(int idx);
int getIdxForFreqInMhz(uint16_t freqVal);
uint8_t freqInMhzToNearestFreqIdx(uint16_t freqVal, boolean upFlag);
void wait_rssi_ready();
void set_time_of_tune();
uint16_t readRSSI();
uint16_t readRSSI(char receiver);
void setReceiver(uint8_t receiver);
void setChannelByIdx(uint8_t freqIdx);
void setChannelByFreq(uint16_t freqInMhz);


#endif /* RX5808FNS_H_ */
