/********************************************************************************
 * MOSES telemetry downlink test code
 *
 * Author: Jacob Plovanic
 * History:
 *  Created Dec 17 2013
 *  Tested  Dec 18 2013 (See results below)
 *
 * Uses the Microgate USB SyncLink adapter to achieve synchronous serial speeds
 * of 10 Mbps to downlink MOSES science data. The data are contained in 16 MB 
 * (possibly 12 MB) files.
 *
 * This program will configure the SyncLink device for use as a linux serial node
 * with the appropriate framing, bitrate, desired error checking, idle pattern,
 * and preamble. It will then send out the 16 MB .roe file. As a first step, we 
 * will use this program to test the timing and capabilities of sending entire
 * MOSES images through the telemetry, since there might be delays associated with
 * the USB transfer. We can then fold this program into the main flight software
 * program.
 *
 * Required files:
 *  synclink.h
 *
 * Compiling for the TS7600 ARM9 flight computer requires the appropriate version
 * of gcc, which is native to the FC's Debian distribution or included in the cross-
 * compiling toolchain available from EmbeddedArm.
 *
 * Compiling sequence:
 *  Cross Compiling:
 *    (CC Toolchain Directory)/arm-fsl-linux-gnueabi-gcc -mtune=arm9 -ggdb3 sendTM.c
 *      -o sendTM
 *
 *  Native Compiling:
 *    gcc -mtune=arm9 -ggdb3 sendTM.c -o sendTM
 *
 * Test Results:
 *  ./sendTM /dev/ttyUSB0 /home/ts-7600-linux/jake/36image.bin 
 * 	fsynth device=/dev/ttyUSB0
 * 	USB device detected
 * 	Found programming information for output frequency = 20000000Hz
 * 	send HDLC data on /dev/ttyUSB0
 * 	Turn on RTS and DTR serial outputs
 * 	Sending data...
 * 	all data sent
 * 	Sent 16794624 bytes of data from file /home/ts-7600-linux/jake/36image.bin.
 * 	Time elapsed: 13.51 seconds.
 * 	Bytes from the 10th write printed (as ASCII characters): $
 * 	Turn off RTS and DTR
 *
 *  The 13.51 seconds is consistent with a 10 Mbps data rate for 16 MB of data. It
 *  looks like there is very little time used to get the data through the USB. The
 *  16794624 bytes sent is larger than the size of the file by 17408 bytes (Debian
 *  reports that the size of 36image.bin is 16777216 bytes). This 
 *  might be due to how fgets() interprets certain characters when it fills the data
 *  buffer for writing, e.g. it's adding an extraneous character to the data buffer
 *  that's not actually in the file. The data file used is repeated instances of
 *  (short)36, which in binary is 0000000000100100 and in char is NULL $. I tried to
 *  print out the data array as a string, but since strings are NULL terminated arrays
 *  the printf() call thinks that there's only one $ in the data buffer when there are
 *  512.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <memory.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>

#include "synclink.h"

#ifndef N_HDLC
#define N_HDLC 13
#endif

#ifndef BUFSIZ
#define BUFSIZ 1024
#endif

/*Function to demonstrate correct command line input*/
void display_usage(void) {
    printf("Usage: sendTM <devname> <imagename>\n"
            "devname = device name (optional) (e.g. /dev/ttyUSB0 etc. "
            "Default is /dev/ttyUSB0)\n"
            "imagename = name of binary file to be sent over TM (e.g. image.roe)\n");
}

/*Program entry point*/
int main(int argc, char ** argv) {

    int fd;
    int rc;
    int sigs, idle;
    int i;
    int ldisc = N_HDLC;
    MGSL_PARAMS params;
    int size = 1024;
    unsigned char databuf[1025]; //RTS changed buffer from 1024 to account for null chars
    unsigned char temp[1025]; //RTS changed buffer from 1024 to account for null chars
    unsigned char endbuf[] = "smart";
    char *devname;
    char *imagename;
    FILE *fp;
    int count = 0; //Number to determine how much data is sent
    struct timeval time_begin, time_end;
    int time_elapsed;

    char* imagepath = "/home/ts-7600-linux/roysmart/images/";
    char* xmlfile = "/home/ts-7600-linux/roysmart/images/imageindex.xml";
    char* image0 = "36image.bin";
    char* image1 = "/home/ts-7600-linux/roysmart/images/080206120404.roe";
    char* image2 = "/home/ts-7600-linux/roysmart/images/080206120411.roe";

    char* images[] = {image0, image1, image2};
    int imageAmount = 3;
    
    
    
    /*Check for correct arguments*/
    if (argc > 3 || argc < 2) {
        printf("Incorrect number of arguments\n");
        display_usage();
        return 1;
    }

    /*Set device name, either from command line or use default value*/
    if (argc == 3)
        devname = argv[1];
    else
        devname = "/dev/ttyUSB0"; //Set the default name of the SyncLink device

    /*Set image filename from command line*/
    if (argc == 3)
        imagename = argv[2];
    else
        imagename = argv[1];

    /* Fork and exec the fsynth program to set the clock source on the SyncLink
     * to use the synthesized 20 MHz clock from the onboard frequency synthesizer
     * chip, for accurate generation of a 10 Mbps datastream. fsynth needs to be
     * in the PATH. 
     */

    pid_t pid = fork();

    if (pid == -1) {
        perror("Fork failure");
        exit(EXIT_FAILURE);
    }

    if (pid == 0) {
        execlp("fsynth", "fsynth", devname, (char *) NULL); //fsynth was compiled with 20MHz
        perror("execlp"); //selected in code
        _exit(EXIT_FAILURE); //Child should die after exec call. If it gets
        //here then the exec failed
    } else if (pid > 0) {
        wait(); //Wait for child to finish
    }

    printf("send HDLC data on %s\n", devname);

    /* open serial device with O_NONBLOCK to ignore DCD input */
    fd = open(devname, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        printf("open error=%d %s\n", errno, strerror(errno));
        return fd;
    }

    /*
     * set N_HDLC line discipline
     *
     * A line discipline is a software layer between a tty device driver
     * and user application that performs intermediate processing,
     * formatting, and buffering of data.
     */
    rc = ioctl(fd, TIOCSETD, &ldisc);
    if (rc < 0) {
        printf("set line discipline error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }

    /* get current device parameters */
    rc = ioctl(fd, MGSL_IOCGPARAMS, &params);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCGPARAMS) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }

    /*
     * modify device parameters
     *
     * HDLC/SDLC mode, loopback disabled (external loopback connector), NRZIs encoding
     * Data transmit clock sourced from BRG
     * Output 10000000bps clock on auxclk output
     * No hardware CRC
     */

    params.mode = MGSL_MODE_HDLC;
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_RXCPIN + HDLC_FLAG_TXC_BRG;
    params.encoding = HDLC_ENCODING_NRZ;
    params.clock_speed = 10000000;
    params.crc_type = HDLC_CRC_16_CCITT | HDLC_CRC_RETURN_EX;
    ;
    params.preamble = HDLC_PREAMBLE_PATTERN_ONES;
    params.preamble_length = HDLC_PREAMBLE_LENGTH_16BITS;

    /* set current device parameters */
    rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCSPARAMS) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }

    /* set transmit idle pattern (sent between frames) */
    idle = HDLC_TXIDLE_ALT_ZEROS_ONES;
    rc = ioctl(fd, MGSL_IOCSTXIDLE, idle);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCSTXIDLE) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }



        /* set device to blocking mode for reads and writes */
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);

        printf("Turn on RTS and DTR serial outputs\n");
        sigs = TIOCM_RTS + TIOCM_DTR;
        rc = ioctl(fd, TIOCMBIS, &sigs);
        if (rc < 0) {
            printf("assert DTR/RTS error=%d %s\n",
                    errno, strerror(errno));
            return rc;
        }

        /* Write imagefile to TM. This requires reading a set number of bytes (1024 currently)
         * from the file into the data buffer, then sending the data buffer to the device 
         * via a write call.
         */
    int j;
    for (j= 0; j < imageAmount * 2; j++) {

        if(j % 2 == 0){
            imagename = images[j/2];
        }
        else{
            imagename = xmlfile;
        }
        
        /*Open image file for reading into a buffered stream*/
        fp = fopen(imagename, "r");
        if (fp == NULL) {
            printf("fopen(%s) error=%d %s\n", imagename, errno, strerror(errno));
            return 1;
        }
        /*Buffer the stream using the standard system bufsiz*/
        rc = setvbuf(fp, NULL, _IOFBF, BUFSIZ);
        if (rc != 0) {
            printf("setvbuf error=%d %s\n", errno, strerror(errno));
            return rc;
        }
        printf("Sending data...\n");
        gettimeofday(&time_begin, NULL); //Determine elapsed time for file write to TM
        while (fgets(databuf, size + 1, fp) != NULL) { //RTS changed buffer from 1024 to account for null chars
            if (count == 10) memcpy(temp, databuf, size); //Store the contents of databuf
            //into the temp buffer
            rc = write(fd, databuf, size);
            if (rc < 0) {
                printf("write error=%d %s\n", errno, strerror(errno));
                break;
            }
            /* block until all data sent */
            rc = tcdrain(fd);
            count++;
        }
        if (rc < 0) return rc; //Finishes the write error handling after the break
//        rc = write(fd, endbuf, 5);
//        if (rc < 0) {
//                printf("write error=%d %s\n", errno, strerror(errno));
//                break;
//            }
//        /* block until all data sent */
//            rc = tcdrain(fd);
        

        gettimeofday(&time_end, NULL); //Timing
        printf("all data sent\n");
        printf("Sent %d bytes of data from file %s.\n", count*size, imagename);
        time_elapsed = 1000000 * ((long) (time_end.tv_sec) - (long) (time_begin.tv_sec))
                + (long) (time_end.tv_usec) - (long) (time_begin.tv_usec);
        printf("Time elapsed: %-3.2f seconds.\n", (float) time_elapsed / (float) 1000000);
        
        sleep(2);
        fclose(fp);
    }
    /*
     * keep auxclk clock output active for 2 seconds to give remote receiver
     * clock cycles for internal processing of received data.
     * If an external device supplies data clocks, this is not needed.
     */
    sleep(2);

    /* Check last entries in databuf to get an idea of the data sent from the image
     * file.
     */

    printf("Bytes from the 10th write printed (as ASCII characters): %s\n", temp);

    printf("Turn off RTS and DTR\n");
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIC, &sigs);
    if (rc < 0) {
        printf("negate DTR/RTS error=%d %s\n", errno, strerror(errno));
        return rc;
    }

    /* Close the device and the image file*/
    close(fd);


    return 0;
}