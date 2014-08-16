#ifndef NANOTECERRORS_H
#define NANOTECERRORS_H

static const char * const NanotecErrorNumber[] =
{
    "No Error",
    "Input Voltage too high",
    "Output Current too high",
    "Input Voltage too low",
    "Can Bus Error",
    "Motor turns even though it is blocked",
    "NMT Master Nodeguarding Timeout",
    "Encoder Defekt",
    "Index Tick from Encoder not found during Auto Setup",
    "Encoder Error in A/B Track",
    "Positive Limit Switch activated and tolerance exceeded",
    "Negative Limit Switch activated and tolerance exceeded",
    "Temperature above 80 degrees",
    "Severe Following Error"
};

#endif // NANOTECERRORS_H
