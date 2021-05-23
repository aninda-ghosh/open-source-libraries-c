/*
 * ringBuffer.h
 *
 *  Created on: 06-May-2021
 *      Author: aninda
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "main.h"
#include "usart.h"

/**
 * The size of a ring buffer.
 * Due to the design only <tt> RING_BUFFER_SIZE-1 </tt> items
 * can be contained in the buffer.
 * The buffer size must be a power of two.
 */
#define RING_BUFFER_SIZE (uint16_t)128
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1)

class RINGBUFFER {
private:
	typedef uint16_t ring_buffer_size_t;
	typedef uint8_t ring_buffer_data_size_t;

	ring_buffer_data_size_t buffer[RING_BUFFER_SIZE];

	ring_buffer_size_t tail_index;
	ring_buffer_size_t head_index;

public:
	RINGBUFFER();

	void enqueue(ring_buffer_data_size_t data);

	void enqueue_arr(ring_buffer_data_size_t *data, ring_buffer_size_t size);

	ring_buffer_data_size_t dequeue(ring_buffer_data_size_t *data);

	ring_buffer_size_t dequeue_arr(ring_buffer_data_size_t *data,
			ring_buffer_size_t len);

	int peek(ring_buffer_size_t index);

	void print(void);

	bool is_empty(void) {
		return (head_index == tail_index);
	}

	bool is_full(void) {
		return ((head_index - tail_index) & RING_BUFFER_MASK)
				== RING_BUFFER_MASK;
	}

	ring_buffer_size_t num_items(void) {
		return ((head_index - tail_index) & RING_BUFFER_MASK);
	}
};

#endif /* INC_RINGBUFFER_H_ */
