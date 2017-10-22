/*
 * Rx5808Fns.h
 *
 */

#ifndef RX5808FNS_H_
#define RX5808FNS_H_

uint8_t channel_from_index(uint8_t channelIndex);
uint8_t getChannelSortTableEntry(int idx);
void wait_rssi_ready();
void set_time_of_tune();
uint16_t readRSSI();
uint16_t readRSSI(char receiver);
void setReceiver(uint8_t receiver);
void setChannelModule(uint8_t channel);


#endif /* RX5808FNS_H_ */
