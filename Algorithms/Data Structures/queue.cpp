/*
 * queue.cpp
 *
 *  Created on: 04-May-2021
 *      Author: aninda
 */

#include "queue.h"

QUEUE::QUEUE(unsignedtype maxElements, unsignedtype elementsSize) {
	elements = (unsignedtype**) malloc(maxElements * sizeof(unsignedtype*));
	for (unsignedtype r = 0; r < maxElements; r++) {
		elements[r] = (unsignedtype*) malloc(
				elementsSize * sizeof(unsignedtype));
	}
	size = 0;
	capacity = maxElements;
	front = 0;
	rear = -1;
}

unsignedtype QUEUE::noOfElements(void) {
	return size;
}

signedtype QUEUE::enqueue(unsignedtype *src, unsignedtype arraysize) {
	/* If the Queue is full, we cannot push an element into it as there is no space for it.*/
	if (size == capacity) {
		return -1;
	} else {
		size++;
		rear = rear + 1;
		/* As we fill the queue in circular fashion */
		if (rear == capacity) {
			rear = 0;
		}
		/* Insert the element in its rear side */
		for (unsignedtype r = 0; r < arraysize; r++) {
			elements[rear][r] = src[r];
		}
	}
	return 0;
}

signedtype QUEUE::dequeue(void) {
	/* If Queue size is zero then it is empty. So we cannot pop */
	if (size == 0) {
		return -1;
	}
	/* Removing an element is equivalent to incrementing index of front by one */
	else {
		size--;
		front++;
		/* As we fill elements in circular fashion */
		if (front == capacity) {
			front = 0;
		}
	}
	return 0;
}

signedtype QUEUE::peek(unsignedtype *dest, unsignedtype arraysize) {
	if (size == 0) {
		return -1;
	}
	/* Return the element which is at the front*/
	for (unsignedtype r = 0; r < arraysize; r++) {
		dest[r] = elements[front][r];

	}
	return 0;
}
