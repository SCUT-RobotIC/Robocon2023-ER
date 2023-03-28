#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

/* remote data struct */
typedef struct _RemoteData_TypeDef
{
	uint16_t size;
	uint16_t flags;
	uint16_t xpos;
	uint16_t ypos;
	uint16_t zpos;
	uint16_t rpos;
	uint16_t upos;
	uint16_t vpos;
	uint16_t buttons;
	uint16_t buttonNumbers;
	uint16_t pov;
	uint16_t reserved1;
	uint16_t reserved2;
}RemoteData;

#endif



