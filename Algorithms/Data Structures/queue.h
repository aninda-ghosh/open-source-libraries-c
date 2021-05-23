/*
 * queue.h
 *
 *  Created on: 04-May-2021
 *      Author: aninda
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include "main.h"

//Use this to change the return type.
#define unsignedtype uint8_t
#define signedtype int8_t

class QUEUE {
public:
	QUEUE(unsignedtype maxElements, unsignedtype elementsSize);
	unsignedtype noOfElements(void);
	signedtype enqueue(unsignedtype *src, unsignedtype arraysize);
	signedtype dequeue(void);
	signedtype peek(unsignedtype *dest, unsignedtype arraysize);

private:
	unsignedtype capacity;
	unsignedtype size;
	unsignedtype front;
	unsignedtype rear;
	unsignedtype **elements;
};

#endif /* INC_QUEUE_H_ */
