/*
 * ringBuffer.cpp
 *
 *  Created on: 06-May-2021
 *      Author: aninda
 */

#include "ringBuffer.h"

RINGBUFFER::RINGBUFFER()
{
    tail_index = 0;
    head_index = 0;
}

void RINGBUFFER::enqueue(ring_buffer_data_size_t data)
{
    /* Is buffer full? */
    if (is_full())
    {
        /* Is going to overwrite the oldest byte */
        /* Increase tail index */
        tail_index = ((tail_index + 1) & RING_BUFFER_MASK);
    }

    /* Place data in buffer */
    buffer[head_index] = data;
    head_index = ((head_index + 1) & RING_BUFFER_MASK);
}

void RINGBUFFER::enqueue_arr(ring_buffer_data_size_t *data, ring_buffer_size_t size)
{
    /* Add bytes; one by one */
    ring_buffer_size_t i;
    for (i = 0; i < size; i++)
    {
        enqueue(data[i]);
    }
}

RINGBUFFER::ring_buffer_data_size_t RINGBUFFER::dequeue(ring_buffer_data_size_t *data)
{
    if (is_empty())
    {
        /* No items */
        return 0;
    }
    if(data != NULL)
        *data = buffer[tail_index];
    tail_index = ((tail_index + 1) & RING_BUFFER_MASK);
    return 1;
}

RINGBUFFER::ring_buffer_size_t RINGBUFFER::dequeue_arr(ring_buffer_data_size_t *data, ring_buffer_size_t len)
{
    if (is_empty())
    {
        /* No items */
        return 0;
    }

    ring_buffer_data_size_t *data_ptr = data;
    ring_buffer_size_t cnt = 0;
    while ((cnt < len) && dequeue(data_ptr))
    {
        cnt++;
        data_ptr++;
    }
    return cnt;
}

int RINGBUFFER::peek(ring_buffer_size_t index)
{
    if (index >= num_items())
    {
        /* No items at index */
        return -1;
    }
    /* Add index to pointer */
    ring_buffer_size_t data_index = ((tail_index + index) & RING_BUFFER_MASK);
//    BM62_DEBUG("[%d, %d] [0x%02X]\n", index, data_index, buffer[data_index]);
    return buffer[data_index];
}

void RINGBUFFER::print(void) {
    int idx = 0;
    SIMPLE_DEBUG("\nPrinting: [");
    while(peek(idx) > -1) {
        /* Print contents */
    	SIMPLE_DEBUG("0x%02X ", peek(idx));
        idx++;
    }
    SIMPLE_DEBUG("]\nNum of items [%d]\n\n", num_items());
}



