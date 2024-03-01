#include "../include/misc.hpp"

int factorial(int number) { 
	if(number <= 1) return 1;
	return number * factorial(number - 1);
}

int sumUpTo(int n) {
    return (n <= 1) ? 1 : (n + sumUpTo(n-1));
}