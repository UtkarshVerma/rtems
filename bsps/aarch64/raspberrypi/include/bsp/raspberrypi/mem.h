#pragma once

typedef volatile unsigned int mem_reg;

#define MEM_REG(addr) (*(mem_reg*)addr)
