#ifdef __cplusplus
extern "C" {
#endif

/*
 * Copyright (C) 2022 Pierantonio Tabaro <toni.tabaro@gmail.com>
 * 
 * pzem16: ModBus RTU client to read PZEM16 power meter registers
 * 
 * 
 * Adapted from:
 * 
 * pzem16: ModBus RTU client to read EASTRON SDM120C smart mini power meter registers
 *
 * Copyright (C) 2022 Pierantonio Tabaro <toni.tabaro@gmail.com>
 * based on: Copyright (C) 2015 Gianfranco Di Prinzio <gianfrdp@inwind.it>
 * 
 * Locking code partially from aurora by Curtronis.
 * Some code by TheDrake too. :)  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

// Enable checks for inter-lock problems debug
#define CHECKFORGHOSTAPPEND     0
#define CHECKFORCLEARLOCKRACE   0

#include <sys/types.h>
#include <sys/file.h>
#include <sys/time.h>

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <getopt.h>
#include <syslog.h>

#if CHECKFORCLEARLOCKRACE
#include <glob.h>
#endif

#include <modbus-version.h>
#include <modbus.h>

#define DEFAULT_RATE 2400

// Read
#define VOLTAGE   0x0000
#define CURRENT   0x0001
#define POWER     0x0003
#define PFACTOR   0x0008
#define FREQUENCY 0x0007
#define TAENERGY  0x0005

// Write
#define DEVICE_ID 0x0002

#define MAX_RETRIES 100

#define RESTART_TRUE  1
#define RESTART_FALSE 0

#define DEBUG_STDERR 1
#define DEBUG_SYSLOG 2

int debug_mask     = 0; //DEBUG_STDERR | DEBUG_SYSLOG; // Default, let pass all
int debug_flag     = 0;
int trace_flag     = 0;

int metern_flag    = 0;

const char *version     = "1.0";
char *programName;
const char *ttyLCKloc   = "/var/lock/LCK.."; /* location and prefix of serial port lock file */

#define CMDLINESIZE 128            /* should be enough for debug */
char cmdline[CMDLINESIZE]="";    
long unsigned int PID;

long unsigned int PPID;
char *PARENTCOMMAND = NULL;

static int yLockWait = 0;          /* Seconds to wait to lock serial port */
static time_t command_delay = -1;  // = 30;  /* MilliSeconds to wait before sending a command */
static time_t settle_time = -1;    // us to wait line to settle before starting chat

char *devLCKfile = NULL;
char *devLCKfileNew = NULL;

void usage(char* program) {
    printf("pzem16 %s: ModBus RTU client to read EASTRON SDM120C smart mini power meter registers\n",version);
    printf("Copyright (C) 2012 Pierantonio Tabaro <toni.tabaro@gmail.com>\n");
    printf("based on: Copyright (C) 2015 Gianfranco Di Prinzio <gianfrdp@inwind.it>\n");
    printf("Complied with libmodbus %s\n\n", LIBMODBUS_VERSION_STRING);
    printf("Usage: %s [-a address] [-d n] [-x] [-p] [-v] [-c] [-e] [-i] [-t] [-f] [-g] [[-m]|[-q]] [-z num_retries] [-j seconds] [-w seconds] [-1 | -2] device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-z num_retries] [-j seconds] [-w seconds] -s new_address device\n", program);
    printf("Required:\n");
    printf("\tdevice\t\tSerial device (i.e. /dev/ttyUSB0)\n");
    printf("\t-a address \tMeter number (1-247). Default: 1\n");
    printf("Reading parameters (no parameter = retrieves all values):\n");
    printf("\t-p \t\tGet power (W)\n");
    printf("\t-v \t\tGet voltage (V)\n");
    printf("\t-c \t\tGet current (A)\n");
    printf("\t-f \t\tGet frequency (Hz)\n");
    printf("\t-g \t\tGet power factor\n");
    printf("\t-t \t\tGet total energy (Wh)\n");
    printf("\t-m \t\tOutput values in IEC 62056 format ID(VALUE*UNIT)\n");
    printf("\t-q \t\tOutput values in compact mode\n");
    printf("Writing new settings parameters:\n");
    printf("\t-s new_address \tSet new meter number (1-247)\n");
    printf("Fine tuning & debug parameters:\n");
    printf("\t-z num_retries\tTry to read max num_retries times on bus before exiting\n");
    printf("\t\t\twith error. Default: 1 (no retry)\n");
    printf("\t-j 1/10 secs\tResponse timeout. Default: 2=0.2s\n");
    printf("\t-D 1/1000 secs\tDelay before sending commands. Default: 0ms\n");
    printf("\t-w seconds\tTime to wait to lock serial port (1-30s). Default: 0s\n");
    printf("\t-W 1/1000 secs\tTime to wait for 485 line to settle. Default: 0ms\n");
    printf("\t-y 1/1000 secs\tSet timeout between every bytes (1-500). Default: disabled\n");
    printf("\t-d debug_level\tDebug (0=disable, 1=debug, 2=errors to syslog, 3=both)\n");
    printf("\t\t\tDefault: 0\n");
    printf("\t-x \t\tTrace (libmodbus debug on)\n");
}

/*--------------------------------------------------------------------------
    tv_diff
----------------------------------------------------------------------------*/
static long inline tv_diff(struct timeval const * const t1, struct timeval const * const t2)
{
    struct timeval res;
    timersub(t1, t2, &res);
    return res.tv_sec*1000000 + res.tv_usec;
}

/*--------------------------------------------------------------------------
        rnd_usleep
----------------------------------------------------------------------------*/
static long inline rnd_usleep(const useconds_t usecs)
{
    long unsigned rnd10 = 10.0*rand()/(RAND_MAX+1.0) + 1;
    if (usleep(usecs*rnd10) == 0)
        return usecs*rnd10;
    else
        return -1;
}

/*--------------------------------------------------------------------------
    getCurTime
----------------------------------------------------------------------------*/
char* getCurTime()
{
    time_t curTimeValue;
    struct tm *ltime;
    static struct timeval _t;
    static struct timezone tz;
    static char CurTime[100];

    time(&curTimeValue);
    ltime = (struct tm *) localtime(&curTimeValue);
    gettimeofday(&_t, &tz);

    sprintf(CurTime, "%04d%02d%02d-%02d:%02d:%02d.%06d", ltime->tm_year + 1900, ltime->tm_mon + 1, ltime->tm_mday, ltime->tm_hour, ltime->tm_min, ltime->tm_sec, (int)_t.tv_usec);

    return CurTime;
}

/*--------------------------------------------------------------------------
    getCmdLine
----------------------------------------------------------------------------*/
void getCmdLine()
{
    int fd = open("/proc/self/cmdline", O_RDONLY);
    int nbytesread = read(fd, cmdline, CMDLINESIZE);
    char *p;
    if (nbytesread>0) {
        for (p=cmdline; p < cmdline+nbytesread; p++) if (*p=='\0') *p=' '; 
        cmdline[nbytesread-1]='\0';
    } else
        cmdline[0]='\0';
    close(fd);
}

/*--------------------------------------------------------------------------
    log_message
----------------------------------------------------------------------------*/
void log_message(const int log, const char* format, ...) {
    va_list args;
    char buffer[1024];
    static int bCmdlineSyslogged = 0;
    
    if (log) {
       va_start(args, format);
       vsnprintf(buffer, 1024, format, args);
       va_end(args);
    }
    
    if (log & debug_mask & DEBUG_STDERR) {
       fprintf(stderr, "%s: %s(%lu) ", getCurTime(), programName, PID);
       fprintf(stderr, "%s", buffer);
       fprintf(stderr, "\n");
    }
    
    if (log & debug_mask & DEBUG_SYSLOG) {
        openlog("pzem16", LOG_PID|LOG_CONS, LOG_USER);
        if (!bCmdlineSyslogged) { 
            char versionbuffer[strlen(programName)+strlen(version)+3];
            snprintf(versionbuffer, strlen(programName)+strlen(version)+3, "%s v%s", programName, version);
            syslog(LOG_INFO, "%s", versionbuffer);
            char parent[80];
            snprintf(parent, sizeof(parent), "parent: %s(%lu)", PARENTCOMMAND, PPID);
            syslog(LOG_INFO, "%s", parent);
            syslog(LOG_INFO, "%s", cmdline);
            bCmdlineSyslogged++;
        }
        syslog(LOG_INFO, buffer);
        closelog();
    }
}

/*--------------------------------------------------------------------------
    getMemPtr
----------------------------------------------------------------------------*/
void *getMemPtr(size_t mSize)
{
    void *ptr;

    ptr = calloc(sizeof(char),mSize);
    if (!ptr) {
        log_message(debug_flag | LOG_SYSLOG, "malloc failed");
        exit(2);
    }
    //cptr = (char *)ptr;
    //for (i = 0; i < mSize; i++) cptr[i] = '\0';
    return ptr;
}

/*--------------------------------------------------------------------------
    ClrSerLock
    Clear Serial Port lock.
----------------------------------------------------------------------------*/
int ClrSerLock(long unsigned int LckPID) {
    FILE *fdserlck, *fdserlcknew;
    long unsigned int PID;
    int bWrite, bRead;
    int errno_save = 0;
    int fLen = 0;
    int cmdLen = 0;
    int curChar = 0;
    char *COMMAND = NULL;

    errno = 0;
    log_message(debug_flag, "devLCKfile: <%s>", devLCKfile);
    log_message(debug_flag, "devLCKfileNew: <%s> ", devLCKfileNew);
    log_message(debug_flag, "Clearing Serial Port Lock (%lu)...", LckPID);
    
    fdserlck = fopen(devLCKfile, "r");
    if (fdserlck == NULL) {
        log_message(debug_flag | DEBUG_SYSLOG, "Problem opening serial device lock file to clear PID %lu: %s for read.",LckPID,devLCKfile);
        return(0);
    }
    log_message(debug_flag, "Acquiring exclusive lock on %s...",devLCKfile);
    flock(fileno(fdserlck), LOCK_EX);   // Will wait to acquire lock then continue
    log_message(debug_flag, "Exclusive lock on %s acquired (%d) %s...",devLCKfile, errno, strerror(errno));

#if CHECKFORCLEARLOCKRACE

    // Check for potential conflicts
    glob_t globbuf;
    int iGlob, fGlob = TRUE;
    
    log_message(debug_flag, "GlobCheck - Check to avoid simultaneous PID clearing");
    for (iGlob=5; iGlob>0 && fGlob; iGlob--) {
      fGlob = FALSE;      
      if (glob("/var/lock/LCK..ttyUSB0.*", GLOB_NOSORT, NULL, &globbuf) != GLOB_NOMATCH) {
          log_message(debug_flag | DEBUG_SYSLOG, "GlobCheck (%u), some other process is clearing lock too!!! (%s)",iGlob, globbuf.gl_pathv[0]);
          fGlob=TRUE;
          log_message(debug_flag, "Sleeping %ldus", rnd_usleep(500000));
      }
      globfree(&globbuf);
    }

#endif

    fdserlcknew = fopen(devLCKfileNew, "a");
    if (fdserlcknew == NULL) {
        log_message(debug_flag | DEBUG_SYSLOG, "Problem opening new serial device lock file to clear PID %lu: %s for write.",LckPID,devLCKfileNew);
        fclose(fdserlck);
        return(0);
    }
    
    // Find cmdLen max len in file
    curChar = 0;
    while (curChar != EOF) {
        fLen = 0;
        while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n' && curChar != ' ') fLen++;
        if (curChar == ' ') {
            fLen = 0;
            while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n') fLen++;
            if (fLen > cmdLen) cmdLen = fLen;
        }
    }
    rewind(fdserlck);
    
    log_message(debug_flag, "cmdLen=%i", cmdLen);
    COMMAND = getMemPtr(cmdLen+1);
    log_message(debug_flag, "cmdLen=%i COMMAND %s", cmdLen, (COMMAND==NULL ? "is null" : "is not null"));
    COMMAND[0] = '\0'; PID = 0;
    errno = 0;
    bRead = fscanf(fdserlck, "%lu%*[ ]%[^\n]\n", &PID, COMMAND);
    errno_save = errno;
    log_message(debug_flag, "errno=%i, bRead=%i LckPID=%lu PID=%lu COMMAND='%s'", errno_save, bRead, LckPID, PID, COMMAND);
    
    while (bRead != EOF && bRead > 0) {
        if (PID != LckPID) {
            errno = 0;
            if (COMMAND[0] != '\0') {
                bWrite = fprintf(fdserlcknew, "%lu %s\n", PID, COMMAND);
                errno_save = errno;
            } else {
                bWrite = fprintf(fdserlcknew, "%lu\n", PID);
                errno_save = errno;
            }
            log_message(debug_flag, "errno=%i, bWrite=%i PID=%lu", errno, bWrite, PID);
            if (bWrite < 0 || errno_save != 0) {
                log_message(debug_flag | DEBUG_SYSLOG, "Problem clearing serial device lock, can't write lock file: %s. %s",devLCKfile,strerror(errno_save));
                log_message(debug_flag | DEBUG_SYSLOG, "(%u) %s",errno_save,strerror(errno_save));
                fclose(fdserlcknew);
                return(0);
            }
        }
        errno=0; PID=0; COMMAND[0] = '\0';
        bRead = fscanf(fdserlck, "%lu%*[ ]%[^\n]\n", &PID, COMMAND);
        errno_save = errno;
        log_message(debug_flag, "errno=%i, bRead=%i LckPID=%lu PID=%lu COMMAND='%s'", errno_save, bRead, LckPID, PID, COMMAND);
    }
    
    fflush(fdserlcknew);

    errno = 0;
    if (rename(devLCKfileNew,devLCKfile)) { 
        log_message(debug_flag | DEBUG_SYSLOG, "Problem clearing serial device lock, can't update lock file: %s.",devLCKfile);
        log_message(debug_flag | DEBUG_SYSLOG, "(%d) %s", errno, strerror(errno));
    }

#if CHECKFORGHOSTAPPEND

    log_message(debug_flag, "Clearing Serial Port Lock almost done...");
    log_message(debug_flag, "Sleeping %luus", rnd_usleep(10000));

    // Check for latest appends (ghost appends)
    int iGhost=10;
    bRead = fscanf(fdserlck, "%lu%*[ ]%*[^\n]\n", &PID);
    while (iGhost > 0) {
        if (bRead > 0) {
            log_message(debug_flag | DEBUG_SYSLOG, "Found ghost append (%d): %s. %lu",iGhost,devLCKfile,PID);
            errno = 0;            
            bWrite = fprintf(fdserlcknew, "%lu\n", PID);
            errno_save = errno;
            if (bWrite < 0 || errno_save != 0) {
                log_message(debug_flag | DEBUG_SYSLOG, "Problem clearing serial device lock, can't write lock file: %s. %s",devLCKfile,strerror (errno_save));
                log_message(debug_flag | DEBUG_SYSLOG, "(%u) %s", errno_save, strerror(errno_save));
                fclose(fdserlcknew);
                return(0);
            }
        }
        fflush(fdserlcknew);
        log_message(debug_flag, "Sleeping %ldus", rnd_usleep(10000));
        iGhost--; PID=0;
        bRead = fscanf(fdserlck, "%lu%*[ ]%*[^\n]\n", &PID);
    }
    
#endif

    fclose(fdserlck);
    fclose(fdserlcknew);
    free(COMMAND);

    log_message(debug_flag, "Clearing Serial Port Lock done");

    return -1;
}

/*--------------------------------------------------------------------------
    AddSerLock
    Queue Serial Port lock intent.
----------------------------------------------------------------------------*/
void AddSerLock(const char *szttyDevice, const char *devLCKfile, const long unsigned int PID, char *COMMAND, const int debug_flag) {
    FILE *fdserlck;
    int bWrite;
    int errno_save = 0;

    log_message(debug_flag, "Attempting to get lock on Serial Port %s...",szttyDevice);
    do {
        fdserlck = fopen((const char *)devLCKfile, "a");
        if (fdserlck == NULL) {
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Problem locking serial device, can't open lock file: %s for write.",devLCKfile);
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Check owner and execution permission for '%s', they shoud be root '-rws--x--x'.",programName);
            exit(2);
        }
        log_message(debug_flag, "Acquiring shared lock on %s...",devLCKfile);
        errno = 0;
        if (flock(fileno(fdserlck), LOCK_SH | LOCK_NB) == 0) break;      // Lock Acquired 
        errno_save=errno;
        
        if (errno_save == EWOULDBLOCK) {
            log_message(debug_flag, "Would block %s, retry (%d) %s...", devLCKfile, errno_save, strerror(errno_save));
            rnd_usleep(25000);
            fclose(fdserlck);
        } else {
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Problem locking serial device, can't open lock file: %s for write. (%d) %s", devLCKfile, errno_save, strerror(errno_save));
            exit(2);
        }
    } while (errno_save == EWOULDBLOCK);
    log_message(debug_flag, "Shared lock on %s acquired...",devLCKfile);
    
    errno=0;
    bWrite = fprintf(fdserlck, "%lu %s\n", PID, COMMAND);
    errno_save = errno;
    fflush(fdserlck);
    fclose(fdserlck);                   // Will release lock
    //fdserlck = NULL;
    if (bWrite < 0 || errno_save != 0) {
        log_message(debug_flag | DEBUG_SYSLOG, "Problem locking serial device, can't write lock file: %s.", devLCKfile);
        log_message(debug_flag | DEBUG_SYSLOG, "(%u) %s", devLCKfile, errno_save, strerror(errno_save));
        exit(2);
    }
}

void exit_error(modbus_t *ctx)
{
/*
      // Wait for line settle
      log_message(debug_flag, "Sleeping %dms for line settle...", settle_time);
      usleep(1000 * settle_time);
      log_message(debug_flag, "Flushed %d bytes", modbus_flush(ctx));
*/
      modbus_close(ctx);
      modbus_free(ctx);
      ClrSerLock(PID);
      free(devLCKfile);
      free(devLCKfileNew);
      if (!metern_flag) {
        printf("NOK\n");
        log_message(debug_flag | DEBUG_SYSLOG, "NOK");
      }
      free(PARENTCOMMAND);
      exit(EXIT_FAILURE);
}

float getMeasureFloat(modbus_t *ctx, int address, int retries, int nb, float divisor) {

    uint16_t tab_reg[nb];
    int rc = -1;
    int i;
    int j = 0;
    int exit_loop = 0;
    int errno_save=0;
    struct timeval tvStart, tvStop;

    while (j < retries && exit_loop == 0) {
      j++;

      if (command_delay) {
        log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
        usleep(command_delay);
      }

      log_message(debug_flag, "%d/%d. Register Address %d [%04X]", j, retries, 30000+address+1, address);
      gettimeofday(&tvStart, NULL); 
      rc = modbus_read_input_registers(ctx, address, nb, tab_reg);
      errno_save = errno;
      gettimeofday(&tvStop, NULL); 

      if (rc == -1) {
        if (trace_flag) fprintf(stderr, "%s: ERROR (%d) %s, %d/%d\n", programName, errno_save, modbus_strerror(errno_save), j, retries);
        log_message(debug_flag | ( j==retries ? DEBUG_SYSLOG : 0), "ERROR (%d) %s, %d/%d, Address %d [%04X]", errno_save, modbus_strerror(errno_save), j, retries, 30000+address+1, address);
        log_message(debug_flag | ( j==retries ? DEBUG_SYSLOG : 0), "Response timeout gave up after %ldus", tv_diff(&tvStop, &tvStart));
        /* libmodbus already flushes 
        log_message(debug_flag, "Flushing modbus buffer");
        log_message(debug_flag, "Flushed %d bytes", modbus_flush(ctx));
        */
        if (command_delay) {
          log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
          usleep(command_delay);
        }
      } else {
        log_message(debug_flag, "Read time: %ldus", tv_diff(&tvStop, &tvStart));
        exit_loop = 1;
      }

    }

    if (rc == -1) {
      exit_error(ctx);
    }

    if (debug_flag) {
       for (i=0; i < rc; i++) {
          log_message(debug_flag, "reg[%d/%d]=%d (0x%X)", i, (rc-1), tab_reg[i], tab_reg[i]);
       }
    }

	if(i==1) {
		tab_reg[1] = 0;
	}

	int32_t tmp = tab_reg[0] | (tab_reg[1] << 16);
    float value = tmp / divisor;

    return value;

}


void changeConfigHex(modbus_t *ctx, int address, int new_value, int restart)
{
    if (command_delay) {
      log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
      usleep(command_delay);
    }

    int n = modbus_write_register(ctx, address, new_value);
    if (n != -1) {
        printf("New value %d for address 0x%X\n", new_value, address);
        if (restart == RESTART_TRUE) printf("You have to restart the meter for apply changes\n");
    } else {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "error 1: (%d) %s, %d, %d", errno, modbus_strerror(errno), n);
        if (errno == EMBXILFUN) // Illegal function
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Tip: is the meter in set mode?");
        exit_error(ctx);
    }
}



/*--------------------------------------------------------------------------
    getIntLen
----------------------------------------------------------------------------*/
int getIntLen(long value){
  long l=!value;
  while(value) { l++; value/=10; }
  return l;
}

/*--------------------------------------------------------------------------
    getPIDcmd
----------------------------------------------------------------------------*/
void *getPIDcmd(long unsigned int PID)
{
    int fdcmd;
    char *COMMAND = NULL;
    size_t cmdLen = 0;
    size_t length;
    char buffer[1024];
    char cmdFilename[getIntLen(PID)+14+1];

    // Generate the name of the cmdline file for the process
    *cmdFilename = '\0';
    snprintf(cmdFilename,sizeof(cmdFilename),"/proc/%lu/cmdline",PID);
    
    // Read the contents of the file
    if ((fdcmd  = open(cmdFilename, O_RDONLY)) < 0) return NULL;
    if ((length = read(fdcmd, buffer, sizeof(buffer))) <= 0) {
        close(fdcmd); return NULL;
    }     
    close(fdcmd);
    
    // read does not NUL-terminate the buffer, so do it here
    buffer[length] = '\0';
    // Get 1st string (command)
    cmdLen=strlen(buffer)+1;
    if((COMMAND = getMemPtr(cmdLen)) != NULL ) {
        strncpy(COMMAND, buffer, cmdLen);
        COMMAND[cmdLen-1] = '\0';
    }

    return COMMAND;
}

/*--------------------------------------------------------------------------
    lockSer
----------------------------------------------------------------------------*/
void lockSer(const char *szttyDevice, const long unsigned int PID, int debug_flag)
{
    char *pos;
    FILE *fdserlck = NULL;
    char *COMMAND = NULL;
    long unsigned int LckPID;
    struct timeval tLockStart, tLockNow;
    int bRead;
    int errno_save = 0;
    int fLen = 0;
    int curChar = 0;
    char *LckCOMMAND = NULL;
    char *LckPIDcommand = NULL;

    pos = strrchr(szttyDevice, '/');
    if (pos > 0) {
        pos++;
        devLCKfile = getMemPtr(strlen(ttyLCKloc)+(strlen(szttyDevice)-(pos-szttyDevice))+1);
        devLCKfile[0] = '\0';
        strcpy(devLCKfile,ttyLCKloc);
        strcat(devLCKfile, pos);
        devLCKfile[strlen(devLCKfile)] = '\0';
        devLCKfileNew = getMemPtr(strlen(devLCKfile)+getIntLen(PID)+2);	/* dot & terminator */
        devLCKfileNew[0] = '\0';
        strcpy(devLCKfileNew,devLCKfile);
        sprintf(devLCKfileNew,"%s.%lu",devLCKfile,PID);
        devLCKfileNew[strlen(devLCKfileNew)] = '\0';
    } else {
        devLCKfile = NULL;
    }

    log_message(debug_flag, "szttyDevice: %s",szttyDevice);
    log_message(debug_flag, "devLCKfile: <%s>",devLCKfile);
    log_message(debug_flag, "devLCKfileNew: <%s>",devLCKfileNew);
    log_message(debug_flag, "PID: %lu", PID);    

    COMMAND = getPIDcmd(PID);
    AddSerLock(szttyDevice, devLCKfile, PID, COMMAND, debug_flag);

    LckPID = 0;
    long unsigned int oldLckPID = 0;
    int staleLockRetries = 0;
    int const staleLockRetriesMax = 2;
    long unsigned int clrStaleTargetPID = 0;    
    int missingPidRetries = 0;
    int const missingPidRetriesMax = 2;
    
    gettimeofday(&tLockStart, NULL);
    tLockNow=tLockStart;

    if (debug_flag) log_message(debug_flag, "Checking for lock");
    while(LckPID != PID && tv_diff(&tLockNow, &tLockStart) <= yLockWait*1000000L) {

        do {
            fdserlck = fopen(devLCKfile, "r");
            if (fdserlck == NULL) {
                log_message(debug_flag | DEBUG_SYSLOG, "Problem locking serial device, can't open lock file: %s for read.",devLCKfile);
                exit(2);
            }
            //log_message(debug_flag, "Acquiring shared lock on %s...",devLCKfile);
            errno = 0;
            if (flock(fileno(fdserlck), LOCK_SH | LOCK_NB) == 0) break;      // Lock Acquired 
            errno_save=errno;
            
            if (errno_save == EWOULDBLOCK) {
                log_message(debug_flag, "Would block %s, retry (%d) %s...", devLCKfile, errno_save, strerror(errno_save));
                rnd_usleep(25000);
                fclose(fdserlck);
            } else {
                log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Problem locking serial device, can't open lock file: %s for read. (%d) %s", devLCKfile, errno_save, strerror(errno_save));
                exit(2);
            }
        } while (errno_save == EWOULDBLOCK);
        //log_message(debug_flag, "Shared lock on %s acquired...",devLCKfile);

        fLen = 0;
        while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n' && curChar != ' ') fLen++;
        fLen = 0;
        if (curChar == ' ') while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n') fLen++;

        rewind(fdserlck);
        
        //if (LckPID != oldLckPID) log_message(debug_flag, "fLen=%i", fLen);
        LckCOMMAND = getMemPtr(fLen+1);
        //if (LckPID != oldLckPID) log_message(debug_flag, "fLen=%i LckCOMMAND %s", fLen, (LckCOMMAND==NULL ? "is null" : "is not null"));
        LckCOMMAND[0] = '\0';
        LckPID=0;
        
        errno = 0;
        bRead = fscanf(fdserlck, "%lu%*[ ]%[^\n]\n", &LckPID, LckCOMMAND);
        errno_save = errno;
        fclose(fdserlck);
        if (LckPID != oldLckPID) {
            log_message(debug_flag | (bRead==EOF || errno_save != 0 ? DEBUG_SYSLOG : 0), "errno=%i, bRead=%i PID=%lu LckPID=%lu", errno_save, bRead, PID, LckPID);
            log_message(debug_flag, "Checking process %lu (%s) for lock", LckPID, LckCOMMAND);
            //oldLckPID = LckPID;
        }
        if (bRead == EOF || LckPID == 0 || errno_save != 0) {
            log_message(debug_flag | DEBUG_SYSLOG, "Problem locking serial device, can't read PID from lock file: %s.",devLCKfile);
            log_message(debug_flag | DEBUG_SYSLOG, "errno=%i, bRead=%i PID=%lu LckPID=%lu", errno_save, bRead, PID, LckPID);
            if (errno_save != 0) {
                // Real error 
                log_message(debug_flag | DEBUG_SYSLOG, "(%u) %s", errno_save, strerror(errno_save));
                free(LckCOMMAND); free(LckPIDcommand); free(COMMAND);
                exit(2);
            } else {
                if (missingPidRetries < missingPidRetriesMax) {
                    missingPidRetries++;
                    log_message(debug_flag, "%s miss process self PID from lock file?",devLCKfile);
                } else if (missingPidRetries >= missingPidRetriesMax) {
                    // Self PID missing... (Should never happen)
                    log_message(debug_flag | DEBUG_SYSLOG, "%s miss process self PID from lock file, amending.",devLCKfile);
                    AddSerLock(szttyDevice, devLCKfile, PID, COMMAND, debug_flag);
                    //LckPID=0;
                    missingPidRetries = 0;
                }
            }
            oldLckPID = LckPID;
        } else { //fread OK
          
          // We got a pid from lockfile, let's clear missing pid status
          missingPidRetries = 0;
          
          LckPIDcommand = getPIDcmd(LckPID);
          
          if (LckPID != oldLckPID) {
              log_message(debug_flag, "PID: %lu COMMAND: \"%s\" LckPID: %lu LckCOMMAND: \"%s\" LckPIDcommand \"%s\"%s", PID, COMMAND
                                          , LckPID, LckCOMMAND, LckPIDcommand
                                          , LckPID == PID ? " = me" : "");
              oldLckPID = LckPID;              
          }
          
//        PID           - this process
//        LckPID        - PID from lock file
//        COMMAND       - this process
//        LckCOMMAND    - process command from lock file
//        LckPIDcommand - process command of process using PID from lock file
          if ((PID != LckPID && LckPIDcommand == NULL) || (LckCOMMAND[0]!='\0' && strcmp(LckPIDcommand,LckCOMMAND) != 0) || strcmp(LckPIDcommand,"") == 0) {
                // Is it a stale lock pid?
                if (staleLockRetries < staleLockRetriesMax) {
                    staleLockRetries++;
                    clrStaleTargetPID = LckPID;
                    log_message(debug_flag | (staleLockRetries > 1 ? DEBUG_SYSLOG : 0), "Stale pid lock(%d)? PID=%lu, LckPID=%lu, LckCOMMAND='%s', LckPIDCommand='%s'", staleLockRetries, PID, LckPID, LckCOMMAND, LckPIDcommand);
                } else if (LckPID == clrStaleTargetPID && staleLockRetries >= staleLockRetriesMax) {
                    log_message(debug_flag | DEBUG_SYSLOG, "Clearing stale serial port lock. (%lu)", LckPID);
                    ClrSerLock(LckPID);
                    staleLockRetries = 0;
                    clrStaleTargetPID = 0;
                }
          } else {
                // Pid lock have a process running, let's reset stale pid retries
                staleLockRetries = 0;
                clrStaleTargetPID = 0;
          } 
        }

        if (yLockWait > 0 && LckPID != PID) {
             rnd_usleep(25000);
             //log_message(debug_flag, "Sleeping %luus", rnd_usleep(25000));
        }

        // Cleanup and loop        
        if (LckCOMMAND != NULL) {
            free(LckCOMMAND);
            LckCOMMAND = NULL;
        }
        if (LckPIDcommand != NULL) {
            free(LckPIDcommand);
            LckPIDcommand = NULL;
        }
        gettimeofday(&tLockNow, NULL);
    } // while
    free(COMMAND);
    if (LckPID == PID) log_message(debug_flag, "Appears we got the lock.");
    if (LckPID != PID) {
        ClrSerLock(PID);
        log_message(DEBUG_STDERR, "Problem locking serial device %s.",szttyDevice);
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Unable to get lock on serial %s for %lu in %ds: still locked by %lu.",szttyDevice,PID,(yLockWait)%30,LckPID);
        log_message(DEBUG_STDERR, "Try a greater -w value (eg -w%u).", (yLockWait+2)%30);
        free(devLCKfile); free(devLCKfileNew); free(PARENTCOMMAND);        
        exit(2);
    }
}

int main(int argc, char* argv[])
{
    int device_address = 1;
    int new_address    = 0;
    int power_flag     = 0;
    int volt_flag      = 0;
    int current_flag   = 0;
    int freq_flag      = 0;
    int pf_flag        = 0;
    int total_flag     = 0;
    int compact_flag   = 0;
    int count_param    = 0;
    int num_retries    = 1;
#if LIBMODBUS_VERSION_MAJOR >= 3 && LIBMODBUS_VERSION_MINOR >= 1 && LIBMODBUS_VERSION_MICRO >= 2
    uint32_t resp_timeout = 2;
    uint32_t byte_timeout = -1;    
#else
    time_t resp_timeout = 2;
    time_t byte_timeout = -1;
#endif
    char *szttyDevice  = NULL;

    int c;
    int read_count     = 0;
   
    programName        = argv[0];

    if (argc == 1) {
        usage(programName);
        exit(EXIT_FAILURE);
    }

    srand(getpid()^time(NULL));      // Init random numbers

    PID = getpid();
    getCmdLine();

    PPID = getppid();
    PARENTCOMMAND = getPIDcmd(PPID); 

    opterr = 0;

    while ((c = getopt (argc, argv, "a:Ab:BcCd:D:efgij:lmM:nN:oOpP:qr:R:s:S:tTvw:W:xy:z:12")) != -1) {
        log_message(debug_flag | DEBUG_SYSLOG, "optind = %d, argc = %d, c = %c, optarg = %s", optind, argc, c, optarg);

        switch (c)
        {
            case 'a':
                device_address = atoi(optarg);

                if (!(0 < device_address && device_address <= 247)) {
                    fprintf (stderr, "%s: Address must be between 1 and 247.\n", programName);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "device_address = %d", device_address);
                break;
            case 'v':
                volt_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "volt_flag = %d, count_param = %d", volt_flag, count_param);
                break;
            case 'p':
                power_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "power_flag = %d, count_param = %d", power_flag, count_param);
                break;
            case 'c':
                current_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "current_flag = %d, count_param = %d", current_flag, count_param);
                break;
            case 't':
                total_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "total_flag = %d, count_param = %d", total_flag, count_param);
                break;
            case 'f':
                freq_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "freq_flag = %d, count_param = %d", freq_flag, count_param);
                break;
            case 'g':
                pf_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "pf_flag = %d, count_param = %d", pf_flag, count_param);
                break;
            case 'd':
                switch (*optarg) {
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                        debug_flag = atoi(optarg) & DEBUG_STDERR;
                        debug_mask = atoi(optarg);
                        break;
                    default:
                         fprintf (stderr, "%s: Debug value must be one of 0,1,2,3.\n", programName);
                         exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "debug_flag = %d", debug_flag);
                break;
            case 'x':
                trace_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "trace_flag = %d, count_param = %d", trace_flag, count_param);
                break;
            case 's':
                new_address = atoi(optarg);
                if (!(0 < new_address && new_address <= 247)) {
                    fprintf (stderr, "%s: New address (%d) out of range, 1-247.\n", programName, new_address);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "new_address = %d, count_param = %d", new_address, count_param);
                break;
            case 'm':
                metern_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "metern_flag = %d, count_param = %d", metern_flag, count_param);
                break;
            case 'q':
                compact_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "compact_flag = %d, count_param = %d", compact_flag, count_param);
                break;
            case 'z':
                num_retries = atoi(optarg);
                if (!(0 < num_retries && num_retries <= MAX_RETRIES)) {
                    fprintf (stderr, "%s: num_retries (%d) out of range, 1-%d.\n", programName, num_retries, MAX_RETRIES);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "num_retries = %d, count_param = %d", num_retries, count_param);
                break;
            case 'j':
                resp_timeout = atoi(optarg);
                if (resp_timeout < 1 || resp_timeout > 500) {
                    fprintf(stderr, "%s: -j Response timeout (%lu) out of range, 0-500.\n",programName,(long unsigned)resp_timeout);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "resp_timeout = %d, count_param = %d", resp_timeout, count_param);
                break;
            case 'y':
                byte_timeout = atoi(optarg);
                if (byte_timeout < 1 || byte_timeout > 500) {
                    fprintf(stderr, "%s: -y Byte timeout (%lu) out of range, 1-500.\n",programName,(long unsigned)byte_timeout);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "byte_timeout = %d, count_param = %d", byte_timeout, count_param);
                break;
            case 'w':
                yLockWait = atoi(optarg);
                if (yLockWait < 1 || yLockWait > 30) {
                    fprintf(stderr, "%s: -w Lock Wait seconds (%d) out of range, 1-30.\n",programName,yLockWait);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "yLockWait = %d, count_param = %d", yLockWait, count_param);
                break;
            case 'W':
                settle_time = atoi(optarg);
                log_message(debug_flag | DEBUG_SYSLOG, "settle_time = %d, count_param = %d", settle_time, count_param);
                break;
            case 'D':
                command_delay = atoi(optarg);
                log_message(debug_flag | DEBUG_SYSLOG, "command_delay = %d, count_param = %d", command_delay, count_param);
                break;
            case '?':
                if (isprint (optopt)) {
                    fprintf (stderr, "%s: Unknown option `-%c'.\n", programName, optopt);
                    usage(programName);
                    exit(EXIT_FAILURE);
                } else {
                    fprintf (stderr,"%s: Unknown option character `\\x%x'.\n",programName, optopt);
                    usage(programName);
                    exit(EXIT_FAILURE);
                }
            default:
                fprintf (stderr, "%s: Unknown option `-%c'.\n", programName, optopt);
                usage(programName);
                exit(EXIT_FAILURE);
        }
    }

    log_message(debug_flag, "cmdline=\"%s\"", cmdline);
        
    if (optind < argc) {               /* get serial device name */
        szttyDevice = argv[optind];
     } else {
        log_message(debug_flag, "optind = %d, argc = %d", optind, argc);
        usage(programName);
        fprintf(stderr, "%s: No serial device specified\n", programName);
        exit(EXIT_FAILURE);
    }

    if (compact_flag == 1 && metern_flag == 1) {
        fprintf(stderr, "%s: Parameter -m and -q are mutually exclusive\n", programName);
        usage(programName);
        exit(EXIT_FAILURE);
    }

    lockSer(szttyDevice, PID, debug_flag);

    modbus_t *ctx;
    
    // Response timeout
    resp_timeout *= 100000;    
    log_message(debug_flag, "resp_timeout=%ldus", resp_timeout);
    
    // Byte timeout
    if (byte_timeout != -1) {
        byte_timeout *= 1000;    
        log_message(debug_flag, "byte_timeout=%ldus", byte_timeout);
    }
    
    // Command delay
    if (command_delay == -1) {
        command_delay = 0;      // default = no command delay
    } else { 
        command_delay *= 1000;        
        log_message(debug_flag, "command_delay=%ldus", command_delay);
    }

    // Settle time delay
    if (settle_time == -1)
        settle_time = 0;        // default = no settle time
    else {
        settle_time *= 1000;
        log_message(debug_flag, "settle_time=%ldus", settle_time);
    }

    //--- Modbus Setup start ---
    
    ctx = modbus_new_rtu(szttyDevice, 9600, 'N', 8, 1);
    if (ctx == NULL) {
        log_message(debug_flag | DEBUG_SYSLOG, "Unable to create the libmodbus context\n");
        ClrSerLock(PID);
        exit(EXIT_FAILURE);
    } else {
        log_message(debug_flag, "Libmodbus context open (9600N1)");
    }

#if LIBMODBUS_VERSION_MAJOR >= 3 && LIBMODBUS_VERSION_MINOR >= 1 && LIBMODBUS_VERSION_MICRO >= 2

    // Considering to get those values from command line
    if (byte_timeout == -1) {
        modbus_set_byte_timeout(ctx, -1, 0);
        log_message(debug_flag, "Byte timeout disabled.");
    } else {
        modbus_set_byte_timeout(ctx, 0, byte_timeout);
        log_message(debug_flag, "New byte timeout: %ds, %dus", 0, byte_timeout);
    }
    modbus_set_response_timeout(ctx, 0, resp_timeout);
    log_message(debug_flag, "New response timeout: %ds, %dus", 0, resp_timeout);

#else

    struct timeval timeout;

    if (byte_timeout == -1) {
        timeout.tv_sec = -1;
        timeout.tv_usec = 0;
        modbus_set_byte_timeout(ctx, &timeout);
        log_message(debug_flag, "Byte timeout disabled.");
    } else {
        timeout.tv_sec = 0;
        timeout.tv_usec = byte_timeout;
        modbus_set_byte_timeout(ctx, &timeout);
        log_message(debug_flag, "New byte timeout: %ds, %dus", timeout.tv_sec, timeout.tv_usec);
    }
    
    timeout.tv_sec = 0;
    timeout.tv_usec = resp_timeout;
    modbus_set_response_timeout(ctx, &timeout);
    log_message(debug_flag, "New response timeout: %ds, %dus", timeout.tv_sec, timeout.tv_usec);

#endif

    //modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL);
    //modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_PROTOCOL);
    modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_NONE);
    
    if (settle_time) {
      // Wait for line settle
      log_message(debug_flag, "Sleeping %ldus for line settle...", settle_time);
      usleep(settle_time);
    }
    
    if (trace_flag == 1) {
        modbus_set_debug(ctx, 1);
    }

    modbus_set_slave(ctx, device_address);

    if (modbus_connect(ctx) == -1) {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Connection failed: (%d) %s\n", errno, modbus_strerror(errno));
        modbus_free(ctx);
        ClrSerLock(PID);
        exit(EXIT_FAILURE);
    }

    //log_message(debug_flag, "Flushed %d bytes", modbus_flush(ctx)); // Already flushed by connect 

    float voltage     = 0;
    float current     = 0;
    float power       = 0;
    float pf          = 0;
    float freq        = 0;
    float tot_energy  = 0;

	if (new_address > 0) {

        log_message(DEBUG_STDERR, "new_address = %d > 0, count_param = %d", new_address, count_param);

        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            // change Address
            log_message(debug_flag, "Before change Address\n");
            changeConfigHex(ctx, DEVICE_ID, new_address, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
    } else if (power_flag   == 0 &&
               volt_flag    == 0 &&
               current_flag == 0 &&
               pf_flag      == 0 &&
               freq_flag    == 0 &&
               total_flag   == 0
       ) {
       // if no parameter, retrieve all values
        power_flag   = 1;
        volt_flag    = 1;
        current_flag = 1;
        freq_flag    = 1;
        pf_flag      = 1;
        total_flag   = 1;
        count_param  = power_flag + volt_flag + 
                       current_flag + freq_flag + pf_flag + 
                       total_flag;
    }

    if (volt_flag == 1) {
        voltage = getMeasureFloat(ctx, VOLTAGE, num_retries, 1, 10.0f);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_V(%3.2f*V)\n", device_address, voltage);
        } else if (compact_flag == 1) {
            printf("%3.2f ", voltage);
        } else {
            printf("Voltage: %3.2f V \n",voltage);
        }
    }

    if (current_flag == 1) {
        current  = getMeasureFloat(ctx, CURRENT, num_retries, 2, 1000.0f);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_C(%3.2f*A)\n", device_address, current);
        } else if (compact_flag == 1) {
            printf("%3.2f ", current);
        } else {
            printf("Current: %3.2f A \n",current);
        }
    }

    if (power_flag == 1) {
        power = getMeasureFloat(ctx, POWER, num_retries, 2, 10.0f);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_P(%3.2f*W)\n", device_address, power);
        } else if (compact_flag == 1) {
            printf("%3.2f ", power);
        } else {
            printf("Power: %3.2f W \n", power);
        }
    }

    if (pf_flag == 1) {
        pf = getMeasureFloat(ctx, PFACTOR, num_retries, 1, 1.0f);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_PF(%3.2f*F)\n", device_address, pf);
        } else if (compact_flag == 1) {
            printf("%3.2f ", pf);
        } else {
            printf("Power Factor: %3.2f \n", pf);
        }
    }

    if (freq_flag == 1) {
        freq = getMeasureFloat(ctx, FREQUENCY, num_retries, 1, 10.0f);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_F(%3.2f*Hz)\n", device_address, freq);
        } else if (compact_flag == 1) {
            printf("%3.2f ", freq);
        } else {
            printf("Frequency: %3.2f Hz \n", freq);
        }
    }

    if (total_flag == 1) {
        tot_energy = getMeasureFloat(ctx, TAENERGY, num_retries, 2, 1.0f);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_TE(%d*Wh)\n", device_address, (int)tot_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)tot_energy);
        } else {
            printf("Total Active Energy: %d Wh \n", (int)tot_energy);
        }
    }


    if (read_count == count_param) {
        // log_message(debug_flag, "Flushed %d bytes", modbus_flush(ctx));
        modbus_close(ctx);
        modbus_free(ctx);
        ClrSerLock(PID);
        free(devLCKfile);
        free(devLCKfileNew);
        free(PARENTCOMMAND);
        if (!metern_flag) printf("OK\n");
    } else {
        exit_error(ctx);
    }

    return 0;
}

#ifdef __cplusplus
}
#endif
