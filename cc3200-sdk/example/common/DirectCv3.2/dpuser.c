/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpuser.c                                                */
/*                                                                          */
/*  Description:    user specific file containing JTAG interface functions  */
/*                  and delay function                                      */
/*                                                                          */
/****************************************************************************/
/* ************ MICROSEMI SOC CORP. DIRECTC LICENSE AGREEMENT ***************/
/* 
PLEASE READ: BEFORE INSTALLING THIS SOFTWARE, CAREFULLY READ THE FOLLOWING 
MICROSEMI SOC CORP LICENSE AGREEMENT REGARDING THE USE OF THIS SOFTWARE. 
INSTALLING THIS SOFTWARE INDICATES THAT YOU ACCEPT AND UNDERSTAND THIS AGREEMENT 
AND WILL ABIDE BY IT. 

Note: This license agreement (“License”) only includes the following software: 
DirectC. DirectC is licensed under the following terms and conditions.

Hereinafter, Microsemi SoC Corp. shall be referred to as “Licensor” or “Author,” 
whereas the other party to this License shall be referred to as “Licensee.” Each 
party to this License shall be referred to, singularly, as a “Party,” or, 
collectively, as the “Parties.”

Permission to use, copy, modify, and/or distribute DirectC for any purpose, with
or without fee, is hereby granted by Licensor to Licensee, provided that the 
above Copyright notice and this permission notice appear in all copies, 
modifications and/or distributions of DirectC.

DIRECTC IS PROVIDED "AS IS" AND THE AUTHOR/LICENSOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO DIRECTC INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS. IN NO EVENT SHALL AUTHOR/LICENSOR BE LIABLE TO LICENSEE FOR ANY DAMAGES, 
INCLUDING SPECIAL, DIRECT,INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES 
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF 
CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION 
WITH THE USE OR PERFORMANCE OF DIRECTC.

Export Control: Information furnished to Licensee may include United States 
origin technical data. Accordingly, Licensee is responsible for complying with, 
and warrants to Licensor that it will comply with, all U.S. export control laws 
and regulations, including the provisions of the Export Administration Act of 
1979 and the Export Administration Regulations promulgated thereunder, the Arms 
Export Control Act, and the sanctions laws administered by the Office of Foreign 
Assets Control including any other U.S. Government regulation applicable to the 
export, re-export, or disclosure of such controlled technical data (or the 
products thereof) to Foreign Nationals, whether within or without the U.S., 
including those employed by, or otherwise associated with, Licensee. Licensee 
shall obtain Licensor’s written consent prior to submitting any request for 
authority to export any such technical data.

ADR: Any dispute between the Parties arising from or related to this License or 
the subject matter hereof, including its validity, construction or performance 
thereunder, shall be exclusively resolved through arbitration by a mutually 
acceptable impartial and neutral arbitrator appointed by the Judicial 
Arbitration and Mediation Services (JAMS) in accordance with its rules and 
procedures. If the Parties are not able to agree on an arbitrator within 10 days 
of the date of request for mediation is served, then JAMS shall appoint an 
arbitrator. Notice of arbitration shall be served and filed with the JAMS main 
offices in Irvine, California. Each Party shall be responsible for all costs 
associated with the preparation and representation by attorneys, or any other 
persons retained thereby, to assist in connection with any such Arbitration. 
However, all costs charged by the mutually agreed upon Arbitration entity shall 
be equally shared by the Parties. The Party seeking Mediation and/or Arbitration 
as provided herein agrees that the venue for any such Mediation and Arbitration 
shall be selected by the other Party and that such venue must be Los Angeles, 
California; New York, New York; or Chicago, Illinois; whereby the applicable law 
and provisions of the Evidence Code of the State selected thereby shall be 
applicable and shall govern the validity, construction and performance of this 
License.

Governing Law: This license will be governed by the laws of the State of 
California, without regard to its conflict of law provisions.

Entire Agreement: This document constitutes the entire agreement between the 
Parties with respect to the subject matter herein and supersedes all other 
communications whether written or oral.
*/

#include "dpuser.h"
#include "dpalg.h"
#include "dputil.h"

#ifdef ENABLE_DISPLAY
#include "uart_if.h"
#endif

/* 
 * User attention:
 * Include files needed to support hardware JTAG interface operations.
 * 
*/

/* This variable is used to select external programming vs IAP programming */
DPUCHAR hardware_interface = GPIO_SEL;
DPUCHAR enable_mss_support = FALSE;

/*
 * User attention:
 * jtag_port_reg:   8 bit Static variable to keep track of the state of all the JTAG pins 
 *                  at all times during the programming operation.
 * Note: User check the variable size to make sure it can cover the hardware IO register. 
 * 
*/
static DPUCHAR jtag_port_reg;
/*
 * User attention: 
 * Module: jtag_inp
 *      purpose: report the logic state of tdo jtag pin
 * Arguments: None
 * Return value: 8 bit value
 *      0, 0x80
 * 
*/
DPUCHAR jtag_inp(void)
{
    return JTAG_GPIO_PIN_VAL(TDO) & TDO ? 0x80u : 0;
}
/*
 * User attention: 
 * Module: jtag_outp
 *      purpose: Set the JTAG port (all JTAG pins)
 * Arguments: 8 bit value containing the new state of all the JTAG pins
 * Return value: None
 * 
*/
void jtag_outp(DPUCHAR outdata)
{
    JTAG_GPIO_PIN_VAL(TMS | TDI | TRST | TCK) = outdata;
}

/*
 * No need to change this function
 * Module: dp_jtag_init
 *      purpose: Set tck and trst pins to logic level one
 * Arguments:
 *      None
 * Return value: None
 * 
*/
void dp_jtag_init(void)
{
    jtag_port_reg = TCK | TRST;
    jtag_outp(jtag_port_reg);
}

/*
 * No need to change this function
 * Module: dp_jtag_tms
 *      purpose: Set tms pin to a logic level one or zero and pulse tck.
 * Arguments: 
 *      tms: 8 bit value containing the new state of tms
 * Return value: None
 * Constraints: Since jtag_outp function sets all the jtag pins, jtag_port_reg is used 
 *              to modify the required jtag pins and preseve the reset.
 * 
*/
void dp_jtag_tms(DPUCHAR tms)        
{   
    jtag_port_reg &= ~(TMS | TCK);
    jtag_port_reg |= (tms ? TMS : 0u);
    jtag_outp(jtag_port_reg);
    jtag_port_reg |= TCK;
    jtag_outp(jtag_port_reg);
}

/*
 * No need to change this function
 * Module: dp_jtag_tms_tdi
 *      purpose: Set tms amd tdi pins to a logic level one or zero and pulse tck.
 * Arguments: 
 *      tms: 8 bit value containing the new state of tms
 *      tdi: 8 bit value containing the new state of tdi
 * Return value: None
 * Constraints: Since jtag_outp function sets all the jtag pins, jtag_port_reg is used 
 *              to modify the required jtag pins and preseve the reset.
 * 
*/
void dp_jtag_tms_tdi(DPUCHAR tms, DPUCHAR tdi)
{
    jtag_port_reg &= ~(TMS | TCK | TDI);
    jtag_port_reg |= ((tms ? TMS : 0u) | (tdi ? TDI : 0u));
    jtag_outp(jtag_port_reg);
    jtag_port_reg |= TCK;
    jtag_outp(jtag_port_reg);
}

/*
 * No need to change this function
 * Module: dp_jtag_tms_tdi_tdo
 *      purpose: Set tms amd tdi pins to a logic level one or zero, 
 *               pulse tck and return tdo level
 * Arguments: 
 *      tms: 8 bit value containing the new state of tms
 *      tdi: 8 bit value containing the new state of tdi
 * Return value: 
 *      ret: 8 bit variable ontaining the state of tdo.
 * Valid return values: 
 *      0x80: indicating a logic level high on tdo
 *      0: indicating a logic level zero on tdo
 * Constraints: Since jtag_outp function sets all the jtag pins, jtag_port_reg is used 
 *              to modify the required jtag pins and preseve the reset.
 * 
*/
DPUCHAR dp_jtag_tms_tdi_tdo(DPUCHAR tms, DPUCHAR tdi)
{
    DPUCHAR ret = 0x80u;
    jtag_port_reg &= ~(TMS | TCK | TDI);
    jtag_port_reg |= ((tms ? TMS : 0u) | (tdi ? TDI : 0u));
    jtag_outp(jtag_port_reg);
    ret = jtag_inp();
    jtag_port_reg |= TCK;
    jtag_outp(jtag_port_reg);
    return ret;
}

/*
 * User attention: 
 * Module: dp_delay
 *      purpose: Execute a time delay for a specified amount of time.
 * Arguments: 
 *      microseconds: 32 bit value containing the amount of wait time in microseconds.
  * Return value: None
 * 
*/
void __attribute__((naked))
asm_delay_loop(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne     asm_delay_loop\n"
          "    bx      lr");
}

void dp_delay(DPULONG microseconds)
{
    asm_delay_loop(CLK_US_LOOP * (unsigned long)(microseconds * 0.84 + 0.5));
}

#ifdef ENABLE_DISPLAY
void dp_display_text(DPCHAR *text)
{
    Report(text);
}

void dp_display_value(DPULONG value, DPUINT descriptive)
{
    if (descriptive == HEX) {
        Report("%lx", value); 
    } else {
        Report("%lu", value);
    }
}
void dp_display_array(DPUCHAR *value, DPUINT bytes, DPUINT descriptive)
{
    int i;
    if (descriptive == HEX) {
        for (i = 0; i < bytes; i++) {
            Report("%02x ", value[bytes - 1 - i]);
        }
    } else {
        for (i = 0; i < bytes; i++) {
            Report("%u ", value[bytes - 1 - i]);
        }
    }
}
#endif
