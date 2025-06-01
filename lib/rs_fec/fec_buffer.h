#pragma once
#include <stdio.h>

class FEC_Block_buffer{
public:

  /**
   * @param packet_len the length of a packet in bytes
   * @param num_blocks the number of blocks to be stored in the buffer
   * @param header_offset the offset for the air2ground struct header
   */
  bool init(size_t packet_len, size_t num_blocks, size_t header_offset, size_t n){
    _MTU = packet_len;
    _num_blocks = num_blocks;
    _header_offset = header_offset;
    _real_MTU = _MTU - _header_offset;

    _block_pointers = (uint8_t**) malloc(sizeof(uint8_t*) * num_blocks); //heap_caps_malloc(packet_len, MALLOC_CAP_SPIRAM);
    if (_block_pointers == NULL) {
      return false;
    }
    for(int i = 0; i < num_blocks; i++) {
      _block_pointers[i] = (uint8_t*) malloc(packet_len);
      if (_block_pointers[i] == NULL) {
        deinit();
      return false;
      }
    }
    
    return true;
  };

  void deinit(){
    for(int i = 0; i < _num_blocks; i++) {
      free(_block_pointers[i]);
    }
    free(_block_pointers);
  };

  uint8_t** get_block_pointers(){
    return _block_pointers;
  };

  uint8_t* get_block_pointer(){
    return _block_pointers[_block_index];
  };

  inline void block_index_add(){
    _block_index ++;
    if(_block_index >= _num_blocks){
      _block_index = 0;
    }
  }

  inline bool fec_buf_ready(){
    return _block_index == _num_blocks -1;
  };

private:
  size_t _MTU;
  size_t _header_offset;
  size_t _real_MTU;

  uint8_t _num_blocks;
  uint8_t** _block_pointers;

  size_t _block_index = 0;
  size_t _data_offset = 0;
};