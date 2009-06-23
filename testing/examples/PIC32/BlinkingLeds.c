#include <plib.h>
int main(void)
{
	int i;
	mPORTDClearBits(BIT_0);
	mPORTDSetPinsDigitalOut(BIT_0);
	mPORTDClearBits(BIT_1);
	mPORTDSetPinsDigitalOut(BIT_1);
	mPORTDClearBits(BIT_2);
	mPORTDSetPinsDigitalOut(BIT_2);

	while (1)
	{
		for (i = 0; i < 500000; i++)
			mPORTDToggleBits(BIT_0);
		for (i = 0; i < 500000; i++)
			mPORTDToggleBits(BIT_1);
		for (i = 0; i < 500000; i++)
			mPORTDToggleBits(BIT_2);
	}

	return 0;
}
