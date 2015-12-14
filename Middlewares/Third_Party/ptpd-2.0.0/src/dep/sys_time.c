/* sys.c */

#include "../ptpd.h"
#include "lwip.h"
#include "main.h"

#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216

void displayStats(const PtpClock *ptpClock)
{
    int leds = 0;

    /* STM_EVAL_LEDOn(LED1); */
//    STM_EVAL_LEDOn(LED2);


    switch (ptpClock->portDS.portState)
    {
    case PTP_INITIALIZING:  leds = 0; break;
    case PTP_FAULTY:        leds = 0; break;
    case PTP_LISTENING:     leds = 0; break;
    case PTP_PASSIVE:       leds = 0; break;
    case PTP_UNCALIBRATED:  leds = 0; break;
    case PTP_SLAVE:         leds = 0; break;
    case PTP_PRE_MASTER:    leds = 2; break;
    case PTP_MASTER:        leds = 2; break;
    case PTP_DISABLED:      leds = 0; break;
    default:                leds = 0; break;
    }

/*    if (leds & 1)
    {
        STM_EVAL_LEDOff(LED1);
    }
*/
//    if (leds & 2)
//    {
//        STM_EVAL_LEDOff(LED2);
//    }

    char buffer[80] = {0};

    const char *s;

    unsigned char * uuid;

    char sign;

    uuid = (unsigned char*)ptpClock->parentDS.parentPortIdentity.clockIdentity;

    /* master clock UUID */
    sprintf(buffer, "\n\r%02X%02X:%02X%02X:%02X%02X:%02X%02X\n\r",
            uuid[0], uuid[1],
            uuid[2], uuid[3],
            uuid[4], uuid[5],
            uuid[6], uuid[7]);

//    LCD_DisplayStringLine(Line4, (uint8_t *)buffer);
    print((uint8_t*)buffer, 79);

    switch (ptpClock->portDS.portState)
    {
    case PTP_INITIALIZING:  s = "init";  break;
    case PTP_FAULTY:        s = "faulty";   break;
    case PTP_LISTENING:     s = "listening";  break;
    case PTP_PASSIVE:       s = "passive";  break;
    case PTP_UNCALIBRATED:  s = "uncalibrated";  break;
    case PTP_SLAVE:         s = "slave";   break;
    case PTP_PRE_MASTER:    s = "pre master";  break;
    case PTP_MASTER:        s = "master";   break;
    case PTP_DISABLED:      s = "disabled";  break;
    default:                s = "?";     break;
    }

    /* state of the PTP */
    sprintf(buffer, "\n\rstate: %s          \n\r", s);

//    LCD_DisplayStringLine(Line5, (uint8_t *)buffer);
    print((uint8_t*)buffer, 79);

    /* one way delay */
    switch (ptpClock->portDS.delayMechanism)
    {
    case E2E:
        sprintf(buffer, "\n\rpath delay: %dns            \n\r", ptpClock->currentDS.meanPathDelay.nanoseconds);
        break;
    case P2P:
        sprintf(buffer, "\n\rpath delay: %dns            \n\r", ptpClock->portDS.peerMeanPathDelay.nanoseconds);
        break;
    default:
        sprintf(buffer, "\n\rpath delay: unknown       \n\r");
        /* none */
        break;
    }
//    LCD_DisplayStringLine(Line6, (uint8_t *)buffer);
    print((uint8_t*)buffer, 79);

    /* offset from master */
    if (ptpClock->currentDS.offsetFromMaster.seconds)
    {
        sprintf(buffer, "\n\roffset: %ds           \n\r", ptpClock->currentDS.offsetFromMaster.seconds);
    }
    else
    {
        sprintf(buffer, "\n\roffset: %dns           \n\r", ptpClock->currentDS.offsetFromMaster.nanoseconds);
    }

//    LCD_DisplayStringLine(Line7, (uint8_t *)buffer);
    print((uint8_t*)buffer, 79);

    /* observed drift from master */
    sign = ' ';

    if (ptpClock->observedDrift > 0) sign = '+';

    if (ptpClock->observedDrift < 0) sign = '-';

    sprintf(buffer, "\n\rdrift: %c%d.%03dppm       \n\r", sign, abs(ptpClock->observedDrift / 1000), abs(ptpClock->observedDrift % 1000));

//    LCD_DisplayStringLine(Line8, (uint8_t *)buffer);
    print((uint8_t*)buffer, 79);

}

void getTime(TimeInternal *time)
{

    struct ptptime_t timestamp;
    ETH_PTPTime_GetTime(&heth, &timestamp);
    time->seconds = timestamp.tv_sec;
    time->nanoseconds = timestamp.tv_nsec;
}

void setTime(const TimeInternal *time)
{

    struct ptptime_t ts;
    ts.tv_sec = time->seconds;
    ts.tv_nsec = time->nanoseconds;

    ETH_PTPTime_SetTime(&heth, &ts);

    DBG("resetting system clock to %ds %dns\n", time->seconds, time->nanoseconds);
}

void updateTime(const TimeInternal *time)
{

    struct ptptime_t timeoffset;

    DBGV("updateTime: %ds %dns\n", time->seconds, time->nanoseconds);

    timeoffset.tv_sec = -time->seconds;
    timeoffset.tv_nsec = -time->nanoseconds;

	/* Coarse update method */
    ETH_PTPTime_UpdateOffset(&heth, &timeoffset);
    DBGV("updateTime: updated\n");
}

UInteger32 getRand(UInteger32 randMax)
{
    return rand() % randMax;
}

Boolean adjFreq(Integer32 adj)
{
    DBGV("adjFreq %d\n", adj);

    if (adj > ADJ_FREQ_MAX)
        adj = ADJ_FREQ_MAX;
    else if (adj < -ADJ_FREQ_MAX)
        adj = -ADJ_FREQ_MAX;

    /* Fine update method */
	ETH_PTPTime_AdjFreq(&heth, adj);

    return TRUE;
}
