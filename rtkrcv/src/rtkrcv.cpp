/*------------------------------------------------------------------------------
* rtkrcv.cpp : based on rtk-gps/gnss receiver console app
*
*          Copyright (C) 2009-2011 by T.TAKASU, All rights reserved.
*          Copyright (C) 2012      by M.STAHL, All rights reserved.
*
* notes   :
*     current version does not support win32 without pthread library
*
* version : $Revision:$ $Date:$
* history : 2009/12/13 1.0  new
*           2010/07/18 1.1  add option -m
*           2010/08/12 1.2  fix bug on ftp/http
*           2011/01/22 1.3  add option misc-proxyaddr,misc-fswapmargin
*           2011/08/19 1.4  fix bug on size of arg solopt arg for rtksvrstart()
*           2012/11/25      use as template for ROS node
*-----------------------------------------------------------------------------*/
#ifndef WIN32
#define _POSIX_C_SOURCE 2
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#ifdef WIN32
#else
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#endif
#include <rtklib/rtklib.h>

#define MAXSTR      1024                /* max length of a string */
#define MAXBUFF     1024                /* max input buffer */
#define MAXRCVCMD   4096                /* max receiver command */
#define NAVIFILE    "rtkrcv.nav"        /* navigation save file */
#define STATFILE    "rtkrcv.stat"       /* solution status file */
#define TRACEFILE   "rtkrcv.trace"      /* trace file */
#define INTKEEPALIVE 1000               /* keep alive interval (ms) */

#define MIN(x,y)    ((x)<(y)?(x):(y))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

/* type definition -----------------------------------------------------------*/
typedef struct {                        /* virtual terminal type */
    int type;                           /* type (0:stdio,1:remote,2:device) */
    int state;                          /* state(0:close,1:open) */
    int in,out;                         /* input/output file descriptor */
    int nbuff;                          /* number of data */
    char buff[MAXBUFF];                 /* input buffer */
    pthread_t svr;                      /* input server */
    pthread_t parent;                   /* parent thread */
    pthread_mutex_t lock;               /* lock flag */
    pthread_cond_t event;               /* event flag */
} vt_t;

/* global variables ----------------------------------------------------------*/
static rtksvr_t svr;                    /* rtk server struct */
static stream_t moni;                   /* monitor stream */

static FILE *logfp      =NULL;          /* log file pointer */

static char passwd[MAXSTR]="admin";     /* login password */
static int timetype     =0;             /* time format (0:gpst,1:utc,2:jst,3:tow) */
static int soltype      =0;             /* sol format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
static int solflag      =2;             /* sol flag (1:std+2:age/ratio/ns) */
static int strtype[]={                  /* stream types */
    STR_SERIAL,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE
};
static char strpath[8][MAXSTR]={""};    /* stream paths */
static int strfmt[]={                   /* stream formats */
    STRFMT_UBX,STRFMT_RTCM3,STRFMT_SP3,SOLF_LLH,SOLF_NMEA
};
static int svrcycle     =10;            /* server cycle (ms) */
static int timeout      =10000;         /* timeout time (ms) */
static int reconnect    =10000;         /* reconnect interval (ms) */
static int nmeacycle    =5000;          /* nmea request cycle (ms) */
static int buffsize     =32768;         /* input buffer size (bytes) */
static int navmsgsel    =0;             /* navigation mesaage select */
static char proxyaddr[256]="";          /* http/ntrip proxy */
static int nmeareq      =0;             /* nmea request type (0:off,1:lat/lon,2:single) */
static double nmeapos[] ={0,0};         /* nmea position (lat/lon) (deg) */
static char rcvcmds[3][MAXSTR]={""};    /* receiver commands files */
static char startcmd[MAXSTR]="";        /* start command */
static char stopcmd [MAXSTR]="";        /* stop command */
static int modflgr[256] ={0};           /* modified flags of receiver options */
static int modflgs[256] ={0};           /* modified flags of system options */
static int moniport     =0;             /* monitor port */
static int keepalive    =0;             /* keep alive flag */
static int fswapmargin  =30;            /* file swap margin (s) */

static prcopt_t prcopt;                 /* processing options */
static solopt_t solopt[2]={{0}};        /* solution options */
static filopt_t filopt  ={""};          /* file options */

/* receiver options table ----------------------------------------------------*/
#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,15:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"

static opt_t rcvopts[]={
    {"console-passwd",  2,  (void *)passwd,              ""     },
    {"console-timetype",3,  (void *)&timetype,           TIMOPT },
    {"console-soltype", 3,  (void *)&soltype,            CONOPT },
    {"console-solflag", 0,  (void *)&solflag,            FLGOPT },
    
    {"inpstr1-type",    3,  (void *)&strtype[0],         ISTOPT },
    {"inpstr2-type",    3,  (void *)&strtype[1],         ISTOPT },
    {"inpstr3-type",    3,  (void *)&strtype[2],         ISTOPT },
    {"inpstr1-path",    2,  (void *)strpath [0],         ""     },
    {"inpstr2-path",    2,  (void *)strpath [1],         ""     },
    {"inpstr3-path",    2,  (void *)strpath [2],         ""     },
    {"inpstr1-format",  3,  (void *)&strfmt [0],         FMTOPT },
    {"inpstr2-format",  3,  (void *)&strfmt [1],         FMTOPT },
    {"inpstr3-format",  3,  (void *)&strfmt [2],         FMTOPT },
    {"inpstr2-nmeareq", 3,  (void *)&nmeareq,            NMEOPT },
    {"inpstr2-nmealat", 1,  (void *)&nmeapos[0],         "deg"  },
    {"inpstr2-nmealon", 1,  (void *)&nmeapos[1],         "deg"  },
    {"outstr1-type",    3,  (void *)&strtype[3],         OSTOPT },
    {"outstr2-type",    3,  (void *)&strtype[4],         OSTOPT },
    {"outstr1-path",    2,  (void *)strpath [3],         ""     },
    {"outstr2-path",    2,  (void *)strpath [4],         ""     },
    {"outstr1-format",  3,  (void *)&strfmt [3],         SOLOPT },
    {"outstr2-format",  3,  (void *)&strfmt [4],         SOLOPT },
    {"logstr1-type",    3,  (void *)&strtype[5],         OSTOPT },
    {"logstr2-type",    3,  (void *)&strtype[6],         OSTOPT },
    {"logstr3-type",    3,  (void *)&strtype[7],         OSTOPT },
    {"logstr1-path",    2,  (void *)strpath [5],         ""     },
    {"logstr2-path",    2,  (void *)strpath [6],         ""     },
    {"logstr3-path",    2,  (void *)strpath [7],         ""     },
    
    {"misc-svrcycle",   0,  (void *)&svrcycle,           "ms"   },
    {"misc-timeout",    0,  (void *)&timeout,            "ms"   },
    {"misc-reconnect",  0,  (void *)&reconnect,          "ms"   },
    {"misc-nmeacycle",  0,  (void *)&nmeacycle,          "ms"   },
    {"misc-buffsize",   0,  (void *)&buffsize,           "bytes"},
    {"misc-navmsgsel",  3,  (void *)&navmsgsel,          MSGOPT },
    {"misc-proxyaddr",  2,  (void *)proxyaddr,           ""     },
    {"misc-fswapmargin",0,  (void *)&fswapmargin,        "s"    },
    
    {"misc-startcmd",   2,  (void *)startcmd,            ""     },
    {"misc-stopcmd",    2,  (void *)stopcmd,             ""     },
    
    {"file-cmdfile1",   2,  (void *)rcvcmds[0],          ""     },
    {"file-cmdfile2",   2,  (void *)rcvcmds[1],          ""     },
    {"file-cmdfile3",   2,  (void *)rcvcmds[2],          ""     },
    
    {"",0,NULL,""}
};

/* discard space characters at tail ------------------------------------------*/
static void chop(char *str)
{
    char *p;
    for (p=str+strlen(str)-1;p>=str&&!isgraph((int)*p);p--) *p='\0';
}
/* thread to send keep alive for monitor port --------------------------------*/
static void *sendkeepalive(void *arg)
{
    trace(3,"sendkeepalive: start\n");
    
    while (keepalive) {
        strwrite(&moni,(unsigned char *)"\r",1);
        sleepms(INTKEEPALIVE);
    }
    trace(3,"sendkeepalive: stop\n");
    return NULL;
}
/* open monitor port ---------------------------------------------------------*/
static int openmoni(int port)
{
    pthread_t thread;
    char path[64];
    
    trace(3,"openmomi: port=%d\n",port);
    
    sprintf(path,":%d",port);
    if (!stropen(&moni,STR_TCPSVR,STR_MODE_RW,path)) return 0;
    strsettimeout(&moni,timeout,reconnect);
    keepalive=1;
    pthread_create(&thread,NULL,sendkeepalive,NULL);
    return 1;
}
/* close monitor port --------------------------------------------------------*/
static void closemoni(void)
{
    trace(3,"closemoni:\n");
    keepalive=0;
    
    /* send disconnect message */
    strwrite(&moni,(unsigned char *)MSG_DISCONN,strlen(MSG_DISCONN));
    
    /* wait fin from clients */
    sleepms(1000);
    
    strclose(&moni);
}
/* read remote console -------------------------------------------------------*/
static int readremote(vt_t *vt, char *buff, int nmax)
{
    int n=0;
    
    trace(4,"readremote:\n");
    
    if (!vt->state) return 0;
    pthread_cond_wait(&vt->event,&vt->lock);
    if (vt->state) {
        n=MIN(nmax,vt->nbuff);
        memcpy(buff,vt->buff,n);
        vt->nbuff=0;
    }
    pthread_mutex_unlock(&vt->lock);
    return n;
}
/* write remote console ------------------------------------------------------*/
static int writeremote(vt_t *vt, char *buff, int n)
{
    char *p,*q,crlf[]="\r\n";;
    
    trace(4,"writeremote: n=%d\n",n);
    
    if (!vt->state) return 0;
    
    for (p=buff;p<buff+n;p=q+1) {
        if (!(q=strchr(p,'\n'))) {
            if (write(vt->out,p,buff+n-p)<buff+n-p) return 0;
            break;
        }
        if (write(vt->out,p,q-p)<q-p) return 0;
        if (write(vt->out,crlf,2)<2) return 0;
    }
    return 1;
}
/* output to console ---------------------------------------------------------*/
static int outvt(vt_t *vt, char *buff, int n)
{
    trace(4,"outvt: n=%d\n",n);
    
    if (logfp) fwrite(buff,n,1,logfp);
    if (vt->type==1) return writeremote(vt,buff,n);
    return write(vt->out,buff,n)==n;
}
/* input line from console ---------------------------------------------------*/
static int inpvt(vt_t *vt, char *buff, int nmax)
{
    int n;
    
    trace(4,"inpvt:\n");
    
    if (vt->type==1) n=readremote(vt,buff,nmax-1);
    else n=read(vt->in,buff,nmax-1);
    if (n<=0) {
        outvt(vt,"\n",1);
        return 0;
    }
    buff[n]='\0';
    if (logfp) fprintf(logfp,"%s",buff);
    chop(buff);
    return 1;
}
/* printf to console ---------------------------------------------------------*/
static int printvt(vt_t *vt, const char *format, ...)
{
    va_list ap;
    int n;
    char buff[MAXSTR];
    
    trace(4,"prvt:\n");
    
    if (!vt->state) {
        va_start(ap,format);
        vfprintf(stderr,format,ap);
        va_end(ap);
        return 1;
    }
    va_start(ap,format);
    n=vsprintf(buff,format,ap);
    va_end(ap);
    return outvt(vt,buff,n);
}
/* confirm overwrite ---------------------------------------------------------*/
static int confwrite(vt_t *vt, const char *file)
{
    FILE *fp;
    char buff[MAXSTR],*p;
    
    strcpy(buff,file);
    if ((p=strstr(buff,"::"))) *p='\0'; /* omit options in path */
    if (!vt->state||!(fp=fopen(buff,"r"))) return 1; /* no existing file */
    fclose(fp);
    printvt(vt,"overwrite %-16s ? (y/n): ",buff);
    if (!inpvt(vt,buff,sizeof(buff))) return 0;
    return toupper((int)buff[0])=='Y';   
}
/* read receiver commands ----------------------------------------------------*/
static int readcmd(const char *file, char *cmd, int type)
{
    FILE *fp;
    char buff[MAXSTR],*p=cmd;
    int i=0;
    
    trace(3,"readcmd: file=%s\n",file);
    
    if (!(fp=fopen(file,"r"))) return 0;
    
    while (fgets(buff,sizeof(buff),fp)) {
        if (*buff=='@') i=1;
        else if (i==type&&p+strlen(buff)+1<cmd+MAXRCVCMD) {
            p+=sprintf(p,"%s",buff);
        }
    }
    fclose(fp);
    return 1;
}
/* read antenna file ---------------------------------------------------------*/
static void readant(vt_t *vt, prcopt_t *opt, nav_t *nav)
{
    const pcv_t pcv0={0};
    pcvs_t pcvr={0},pcvs={0};
    pcv_t *pcv;
    gtime_t time=timeget();
    int i;
    
    trace(3,"readant:\n");
    
    opt->pcvr[0]=opt->pcvr[1]=pcv0;
    if (!*filopt.rcvantp) return;
    
    if (readpcv(filopt.rcvantp,&pcvr)) {
        for (i=0;i<2;i++) {
            if (!*opt->anttype[i]) continue;
            if (!(pcv=searchpcv(0,opt->anttype[i],time,&pcvr))) {
                printvt(vt,"no antenna %s in %s",opt->anttype[i],filopt.rcvantp);
                continue;
            }
            opt->pcvr[i]=*pcv;
        }
    }
    else printvt(vt,"antenna file open error %s",filopt.rcvantp);
    
    if (readpcv(filopt.satantp,&pcvs)) {
        for (i=0;i<MAXSAT;i++) {
            if (!(pcv=searchpcv(i+1,"",time,&pcvs))) continue;
            nav->pcvs[i]=*pcv;
        }
    }
    else printvt(vt,"antenna file open error %s",filopt.satantp);
    
    free(pcvr.pcv); free(pcvs.pcv);
}
/* start rtk server ----------------------------------------------------------*/
static int startsvr(vt_t *vt)
{
    double pos[3],npos[3];
    char s[3][MAXRCVCMD]={"","",""},*cmds[]={NULL,NULL,NULL};
    char *ropts[]={"","",""};
    char *paths[]={
        strpath[0],strpath[1],strpath[2],strpath[3],strpath[4],strpath[5],
        strpath[6],strpath[7]
    };
    int i,ret,stropt[8]={0};
    
    trace(3,"startsvr:\n");
    
    /* read start commads from command files */
    for (i=0;i<3;i++) {
        if (!*rcvcmds[i]) continue;
        if (!readcmd(rcvcmds[i],s[i],0)) {
            printvt(vt,"no command file: %s\n",rcvcmds[i]);
        }
        else cmds[i]=s[i];
    }
    /* confirm overwrite */
    for (i=3;i<8;i++) {
        if (strtype[i]==STR_FILE&&!confwrite(vt,strpath[i])) return 0;
    }
    if (prcopt.refpos==4) { /* rtcm */
        for (i=0;i<3;i++) prcopt.rb[i]=0.0;
    }
    pos[0]=nmeapos[0]*D2R;
    pos[1]=nmeapos[1]*D2R;
    pos[2]=0.0;
    pos2ecef(pos,npos);
    
    /* read antenna file */
    readant(vt,&prcopt,&svr.nav);
    
    /* open geoid data file */
    if (solopt[0].geoid>0&&!opengeoid(solopt[0].geoid,filopt.geoid)) {
        trace(2,"geoid data open error: %s\n",filopt.geoid);
        printvt(vt,"geoid data open error: %s\n",filopt.geoid);
    }
    for (i=0;*rcvopts[i].name;i++) modflgr[i]=0;
    for (i=0;*sysopts[i].name;i++) modflgs[i]=0;
    
    /* set stream options */
    stropt[0]=timeout;
    stropt[1]=reconnect;
    stropt[2]=1000;
    stropt[3]=buffsize;
    stropt[4]=fswapmargin;
    strsetopt(stropt);
    
    if (strfmt[2]==8) strfmt[2]=STRFMT_SP3;
    
    /* set ftp/http directory and proxy */
    strsetdir(filopt.tempdir);
    strsetproxy(proxyaddr);
    
    /* execute start command */
    if (*startcmd&&(ret=system(startcmd))) {
        trace(2,"command exec error: %s (%d)\n",startcmd,ret);
        printvt(vt,"command exec error: %s (%d)\n",startcmd,ret);
    }
    /* start rtk server */
    if (!rtksvrstart(&svr,svrcycle,buffsize,strtype,paths,strfmt,navmsgsel,
                     cmds,ropts,nmeacycle,nmeareq,npos,&prcopt,solopt,&moni)) {
        trace(2,"rtk server start error\n");
        printvt(vt,"rtk server start error\n");
        return 0;
    }
    return 1;
}
/* stop rtk server -----------------------------------------------------------*/
static void stopsvr(vt_t *vt)
{
    char s[3][MAXRCVCMD]={"","",""},*cmds[]={NULL,NULL,NULL};
    int i,ret;
    
    trace(3,"stopsvr:\n");
    
    if (!svr.state) return;
    
    /* read stop commads from command files */
    for (i=0;i<3;i++) {
        if (!*rcvcmds[i]) continue;
        if (!readcmd(rcvcmds[i],s[i],1)) {
            printvt(vt,"no command file: %s\n",rcvcmds[i]);
        }
        else cmds[i]=s[i];
    }
    /* stop rtk server */
    rtksvrstop(&svr,cmds);
    
    /* execute stop command */
    if (*stopcmd&&(ret=system(stopcmd))) {
        trace(2,"command exec error: %s (%d)\n",stopcmd,ret);
        printvt(vt,"command exec error: %s (%d)\n",stopcmd,ret);
    }
    if (solopt[0].geoid>0) closegeoid();
}

/* rtkrcv main -----------------------------------------------------------------
* sysnopsis
*     rtkrcv [-m port][-t level][-r level]
*
* description
*     A ROS version of the real-time positioning AP by rtklib. To start
*     or stop RTK server, to configure options or to print solution/status,
*     login a console and input commands. As default, stdin/stdout are used for
*     the console. Use -p option for network login with telnet protocol. To show
*     the available commands, type ? or help on the console. The initial
*     processing options are loaded from ROS parameters. To configure the
*     processing options, create a launch file or use set, load or save command
*     on the console.
*
* option
*     -m port    port number for monitor stream
*     -r level   output solution status file (0:off,1:states,2:residuals)
*     -t level   debug trace level (0:off,1-5:on)
*
*-----------------------------------------------------------------------------*/

static vt_t svr_vt={0};
static int outstat=0,rcv_trace=0;

int loadrosopts(opt_t *opts);

int start_rtkrcv(int argc, char **argv)
{
    int i;

    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-m")&&i+1<argc) moniport=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-r")&&i+1<argc) outstat=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-t")&&i+1<argc) rcv_trace=atoi(argv[++i]);
        else fprintf(stderr,"Unknown option: %s\n",argv[i]);
    }
    if (rcv_trace>0) {
        traceopen(TRACEFILE);
        tracelevel(rcv_trace);
    }
    /* initialize rtk server and monitor port */
    rtksvrinit(&svr);
    strinit(&moni);

    /* load options from ROS */
    resetsysopts();
    loadrosopts(rcvopts);
    loadrosopts(sysopts);
    getsysopts(&prcopt,solopt,&filopt);

    /* read navigation data */
    if (!readnav(NAVIFILE,&svr.nav)) {
        fprintf(stderr,"no navigation data: %s\n",NAVIFILE);
    }
    if (outstat>0) {
        rtkopenstat(STATFILE,outstat);
    }
    /* open monitor port */
    if (moniport>0&&!openmoni(moniport)) {
        fprintf(stderr,"monitor port open error: %d\n",moniport);
        return 0;
    }
    /* start rtk server */
    if (!startsvr(&svr_vt)) return 0;
    return 1;
}

void stop_rtkrcv() {
    /* stop rtk server */
    stopsvr(&svr_vt);

    if (moniport>0) closemoni();

    if (outstat>0) rtkclosestat();

    if (rcv_trace>0) traceclose();

    /* save navigation data */
    if (!savenav(NAVIFILE,&svr.nav)) {
        fprintf(stderr,"navigation data save error: %s\n",NAVIFILE);
    }
}

char get_solution(double &lat, double &lon, double &height)
{
    rtksvrlock(&svr);
    const sol_t *sol = &svr.solbuf[svr.nsol-1];
    const double *rb = svr.rtk.rb;
    double pos[3]={0},Qr[9],Qe[9]={0},dms1[3]={0},dms2[3]={0},bl[3]={0};
    double enu[3]={0},pitch=0.0,yaw=0.0,len;
    int i;
    int status;

    if (sol->time.time == 0 || !sol->stat) {
        rtksvrunlock(&svr);
        return -1; // no fix
    }
    if (1 <= sol->stat && sol->stat <= 2) status = 0; // fix
    if (sol->stat == 3)                   status = 1; // sbas fix
    else                                  status = 2; // differential fix

    if (norm(sol->rr,3) > 0.0 && norm(rb,3) > 0.0) {
        for (i=0;i<3;i++) bl[i]=sol->rr[i]-rb[i];
    }
    len=norm(bl,3);
    Qr[0]=sol->qr[0];
    Qr[4]=sol->qr[1];
    Qr[8]=sol->qr[2];
    Qr[1]=Qr[3]=sol->qr[3];
    Qr[5]=Qr[7]=sol->qr[4];
    Qr[2]=Qr[6]=sol->qr[5];

    if (norm(sol->rr,3) > 0.0) {
        ecef2pos(sol->rr,pos);
        covenu(pos,Qr,Qe);
        lat = pos[0]*R2D;
        lon = pos[1]*R2D;
        if (solopt[0].height == 1) pos[2]-=geoidh(pos); /* geodetic */
        height = pos[2];
    }
    svr.nsol=0;
    rtksvrunlock(&svr);
    return status;
}
