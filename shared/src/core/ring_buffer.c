#include "core/ring_buffer.h"

void ring_buffer_setup(ring_buffer_t *rb, uint8_t *buffer, uint32_t size)
{
    rb->buffer = buffer;
    rb->read_index = 0;
    rb->write_index = 0;
    rb->mask = size - 1; // API assumption: the size is a power of two.
}

bool ring_buffer_empty(ring_buffer_t *rb)
{
    return (rb->read_index == rb->write_index);
}

bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    // First we make a local copy to keep track of the value during the call time (in case an ISR happens)
    uint32_t read_index = rb->read_index;
    uint32_t write_index = rb->write_index;

    uint32_t next_write_index = (write_index + 1) & rb->mask;
    // Avoid having write_index equal to read_index (and therefore making the ring buffer "empty")
    // by discarding the newer incoming data.
    if(next_write_index == read_index) {
        return false;
    }

    rb->buffer[write_index] = data;
    rb->write_index = next_write_index;

    return true;
}

bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    // First we make a local copy to keep track of the value during the call time (in case an ISR happens)
    uint32_t read_index = rb->read_index;
    uint32_t write_index = rb->write_index;

    if(read_index == write_index) {
        return false;
    }

    *data = rb->buffer[read_index];
    read_index = (read_index + 1) & rb->mask;
    rb->read_index = read_index;

    return true;
}
