/********************************************************************************
 * MOSES telemetry downlink test code
 *
 * Author: Jacob Plovanic, Roy Smart
 * History:
 *  Created Dec 17 2013
 *  Tested  Dec 18 2013 
 *  Tested May 9 2014 successfully at White Sands Missile Range (See results below)
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
 *      ./sendtm 
 *       fsynth device=/dev/ttyUSB0
 *       USB device detected
 *       Found programming information for output frequency = 20000000Hz
 *       send HDLC data on /dev/ttyUSB0
 *       Turn on RTS and DTR serial outputs
 *       Sending data...
 *       all data sent
 *       Sent 16777216 bytes of data from file 36image.bin.
 *       Time elapsed: 13.58 seconds.
 *       Sending data...
 *       all data sent
 *       Sent 28672 bytes of data from file /home/ts-7600-linux/roysmart/images/imageindex.xml.
 *       Time elapsed: 0.02 seconds.
 *
 *
 *  The 13.51 seconds is consistent with a 10 Mbps data rate for 16 MB of data. It
 *  looks like there is very little time used to get the data through the USB. The
 *  16777216 bytes sent is the same size as the file reported by Debian. 
 * 
 * In previous implementations of this code, the buffer holding the images was read using the 
 * function fgets(). This function was incorrect as it didn't treat chars the same as all other bytes.
 * Testing revealed that the code was correctly sending chars, but other bytes were being replaced 
 * by random data. This problem was rectified by using the function fread() to parse the buffer in a 
 * binary fashion.
 * 
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
#define BUFSIZ 4096
#endif

/*Function to demonstrate correct command line input*/
void display_usage(void) {
    printf("Usage: sendTM <devname> \n"
            "devname = device name (optional) (e.g. /dev/ttyUSB2 etc. "
            "Default is /dev/ttyUSB0)\n");
}

/*Program entry point*/
int main() {

    int fd, rd, rc;
    int j, k;
    int sigs, idle, itr;
    int ldisc = N_HDLC;
    FILE *fp = NULL;
    MGSL_PARAMS params;
    unsigned char *databuf[BUFSIZ];
    int totalSize = 0;
    long sz;
    unsigned char endbuf[] = "smart"; //Used this string as end-frame to terminate seperate files
    char *devname;
    char *imagename;
    int count = 0; //Number to determine how much data is sent
    int time_elapsed;
    struct timeval time_begin, time_end;

    char* xmlfile = "/home/moses/roysmart/images/imageindex.xml";
    char* image0 = "/home/moses/roysmart/images/080206120404.roe";
    char* image1 = "/home/moses/roysmart/images/080206120411.roe";
    char* image2 = "/home/moses/roysmart/images/080206120418.roe";
    char* image3 = "/home/moses/roysmart/images/080206120428.roe";
    char* image4 = "/home/moses/roysmart/images/080206120440.roe";
    char* image5 = "/home/moses/roysmart/images/080206120458.roe";
    char* image6 = "/home/moses/roysmart/images/080206120529.roe";

    /*image queue*/
    char* images[] = {image0, image1, image2, image3, image4, image5, image6};
    int imageAmount = 14;



    /*Check for correct arguments*/
    //if (argc > 2 || argc < 1) {
        //printf("Incorrect number of arguments\n");
        //display_usage();
        //return 1;
    //}

    /*Set device name, either from command line or use default value*/
    //if (argc == 3)
        //devname = argv[1];
    //else
        devname = "/dev/ttyUSB0"; //Set the default name of the SyncLink device

    /* Fork and exec the fsynth program to set the clock source on the SyncLink
     * to use the synthesized 20 MHz clock from the onboard frequency synthesizer
     * chip, for accurate generation of a 10 Mbps datastream. fsynth needs to be
     * in the PATH. 
     */

    pid_t pid = fork();
    printf("forking process\n");
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
    else printf("device opened on %s\n", devname);

    /*
     * set N_HDLC line discipline						//Change this to N_TTY?
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

    params.mode = MGSL_MODE_HDLC;						//N_TTY?
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_RXCPIN + HDLC_FLAG_TXC_BRG;
    params.encoding = HDLC_ENCODING_NRZ;
    params.clock_speed = 10000000;
    params.crc_type = HDLC_CRC_16_CCITT;
    params.preamble = HDLC_PREAMBLE_PATTERN_ONES;				//Remove?
    params.preamble_length = HDLC_PREAMBLE_LENGTH_16BITS;

    /* set current device parameters */
    rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCSPARAMS) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }

    /* set transmit idle pattern (sent between frames) */
    idle = HDLC_TXIDLE_ALT_ZEROS_ONES;						//Change? consult email stream
    rc = ioctl(fd, MGSL_IOCSTXIDLE, idle);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCSTXIDLE) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }



    /* set device to blocking mode for reads and writes */
    int blk = fcntl(fd, F_GETFL);
    blk = (blk | O_NONBLOCK);
    //fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);

    printf("Turn on RTS and DTR serial outputs\n\n");
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIS, &sigs);
    if (rc < 0) {
        printf("assert DTR/RTS error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }

    /*enable transmitter*/
    int enable = 1;
    rc = ioctl(fd, MGSL_IOCTXENABLE, enable);

    /* Write imagefile to TM. This requires reading the entire image in chunks of 4096 bytes
     * from the file into the data buffer, then sending the each chunk to the device 
     * via a write call.
     */

    for (j = 0; j < imageAmount; j++) {
        count = 0;
        totalSize = 0;
	if (j % 2 == 0) {       //If we are on an odd loop send an image
            sz = 16777200;
            imagename = images[j / 2];
        } else {
            sz = 28165;
            imagename = xmlfile;     //otherwise send an xml file
        }
            
        /*Open image file for reading into a buffered stream*/
        fp = fopen(imagename, "r");
        if (fp == NULL) {
            printf("fopen(%s) error=%d %s\n", imagename, errno, strerror(errno));
            return 1;
        }

	//struct stat st;
        //fseek(fp, 0L, SEEK_END);
	//stat(fp, &st);
        //sz = ftell(fp);
	//fseek(fp, 0L, SEEK_SET);
	itr = (int)((sz + (0.5*BUFSIZ)) / (BUFSIZ));
        //itr = 4;
	printf("New file size: %d Bytes and %d iterations\n", (int)sz, (2 * itr));


//        /*Buffer the stream using the standard system bufsiz*/
//        rc = setvbuf(fp, NULL, _IOFBF, BUFSIZ);
//        if (rc != 0) {
//            printf("setvbuf error=%d %s\n", errno, strerror(errno));
//            return rc;
//        }
        /*Read the image into memory*/
        for (k=0;k<itr;k++) {
            //databuf[k] = malloc((size_t)BUFSIZ);
            rd = fread(databuf, sizeof(char), BUFSIZ, fp);
	    //rd = read(fp, *buf, (size_t)BUFSIZ);
        }

        printf("image: %s read into memory\n", imagename);
	printf("Sending data from memory...\n");

	gettimeofday(&time_begin, NULL); //Determine elapsed time for file write to TM
        
        for (k=0;k<itr;k++) {
            //if (count == 10) memcpy(temp, databuf, size); //Store the contents of databuf into the temp buffer
            rc = write(fd, databuf, BUFSIZ);

	    if (rc < 0) {
                printf("write error=%d %s\n", errno, strerror(errno));
                break;
            }
	    totalSize += rc;

            /* block until all data sent */
            rc = tcdrain(fd);

	    if (rc < 0) {
                printf("write error=%d %s\n", errno, strerror(errno));
                break;
            }
            
            count++;


        }
        if (rc < 0) {
	    printf("write error handling...\n");
	    return rc; //Finishes the write error handling after the break
	}

	fclose(fp);
        rc = write(fd, endbuf, 5);

	if (rc < 0) {
                printf("write error=%d %s\n", errno, strerror(errno));
                break;
         }

	/*block until all data sent*/
        rc = tcdrain(fd);
        if (rc < 0) {
            printf("endbuf write error=%d %s\n", errno, strerror(errno));
            break;
        }

        /*clear the data buffer*/
        //fflush(fp);


        gettimeofday(&time_end, NULL); //Timing
        printf("all data sent\n");
        printf("Sent %d bytes of data from file %s.\n", totalSize, imagename);
        time_elapsed = 1000000 * ((long) (time_end.tv_sec) - (long) (time_begin.tv_sec))
                + (long) (time_end.tv_usec) - (long) (time_begin.tv_usec);
        printf("Time elapsed: %-3.2f seconds.\n\n", (float) time_elapsed / (float) 1000000);

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

    //    printf("Bytes from the 10th write printed (as ASCII characters): %s\n", temp);

    printf("Turn off RTS and DTR\n");
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIC, &sigs);
    if (rc < 0) {
        printf("negate DTR/RTS error=%d %s\n", errno, strerror(errno));
        return rc;
    }

    /* Close the device and the image file*/
    close(fd);
    //close(fp);

    return 0;
}
