#include <stdio.h>
#include <ros/ros.h>
#include <rtklib/rtklib.h>
#include </home/asfandyar/gnss/rtklib/src/src/streamsvr.c> //for raw2rtcm, rtcm2rtcm
#include </home/asfandyar/gnss/rtklib/src/src/rtcm.c>      //for  input_rtcm2, input_rtcm3
#include </home/asfandyar/gnss/rtklib/src/src/rcvraw.c>	   //for input_raw
#include <rtklib/Navigation.h>
#include <rtklib/Observation.h>
#include <rtklib/ObservationData.h>
#include <sstream>
#include <string>

// Function to convert Observation message from raw format to ros
void obsfunc(rtklib::Observation obsarr, rtklib::ObservationData obserdata , strconv_t *conv){
    int i, j;
    for(i=0; i<conv->raw.obs.nmax; i++){
        obserdata.sat=conv->raw.obs.data[i].sat;
        obserdata.rcv=conv->raw.obs.data[i].rcv;

        for(j=0; j<(NFREQ+NEXOBS); j++){
            obserdata.SNR[j]=conv->raw.obs.data[i].SNR[j];
            obserdata.LLI[j]=conv->raw.obs.data[i].LLI[j];
            obserdata.code[j]=conv->raw.obs.data[i].code[j];
            obserdata.L[j]=conv->raw.obs.data[i].L[j];
            obserdata.P[j]=conv->raw.obs.data[i].P[j];
            obserdata.D[j]=conv->raw.obs.data[i].D[j];
        } 
       obsarr.data.push_back(obserdata);  
    }
}


// Function to convert Navigation message from raw format to ros
void navmsgfunc(rtklib::Navigation navigmsg, strconv_t* conv){

    int i, j, p;

    for(i=0; i<conv->raw.nav.nmax; i++){
        navigmsg.eph[i].sat=conv->raw.nav.eph[i].sat;
    }
    for(i=0; i<conv->raw.nav.ngmax; i++){  //time not implemented
        navigmsg.geph[i].sat=conv->raw.nav.geph[i].sat;
        navigmsg.geph[i].iode=conv->raw.nav.geph[i].iode;
        navigmsg.geph[i].frq=conv->raw.nav.geph[i].frq;
        navigmsg.geph[i].svh=conv->raw.nav.geph[i].svh;
        navigmsg.geph[i].sva=conv->raw.nav.geph[i].sva;
        navigmsg.geph[i].age=conv->raw.nav.geph[i].age;
        navigmsg.geph[i].pos.x=conv->raw.nav.geph[i].pos[0];
        navigmsg.geph[i].pos.y=conv->raw.nav.geph[i].pos[1];
        navigmsg.geph[i].pos.z=conv->raw.nav.geph[i].pos[2];
        navigmsg.geph[i].vel.x=conv->raw.nav.geph[i].vel[0];
        navigmsg.geph[i].vel.y=conv->raw.nav.geph[i].vel[1];
        navigmsg.geph[i].vel.z=conv->raw.nav.geph[i].vel[2];
        navigmsg.geph[i].acc.x=conv->raw.nav.geph[i].acc[0];
        navigmsg.geph[i].acc.y=conv->raw.nav.geph[i].acc[1];
        navigmsg.geph[i].acc.z=conv->raw.nav.geph[i].acc[2];
        navigmsg.geph[i].taun=conv->raw.nav.geph[i].taun;
        navigmsg.geph[i].gamn=conv->raw.nav.geph[i].gamn;
        navigmsg.geph[i].dtaun=conv->raw.nav.geph[i].dtaun;
    }
    for(i=0; i<conv->raw.nav.nsmax; i++){  //time not implemented
        navigmsg.seph[i].sat=conv->raw.nav.seph[i].sat;
        navigmsg.seph[i].sva=conv->raw.nav.seph[i].sva;
        navigmsg.seph[i].svh=conv->raw.nav.seph[i].svh;
        navigmsg.seph[i].af0=conv->raw.nav.seph[i].af0;
        navigmsg.seph[i].af1=conv->raw.nav.seph[i].af1;
        
        navigmsg.seph[i].pos.x=conv->raw.nav.seph[i].pos[0];
        navigmsg.seph[i].pos.y=conv->raw.nav.seph[i].pos[1];
        navigmsg.seph[i].pos.z=conv->raw.nav.seph[i].pos[2];

        navigmsg.seph[i].vel.x=conv->raw.nav.seph[i].vel[0];
        navigmsg.seph[i].vel.y=conv->raw.nav.seph[i].vel[1];
        navigmsg.seph[i].vel.z=conv->raw.nav.seph[i].vel[2];

        navigmsg.seph[i].acc.x=conv->raw.nav.seph[i].acc[0];
        navigmsg.seph[i].acc.y=conv->raw.nav.seph[i].acc[1];
        navigmsg.seph[i].acc.z=conv->raw.nav.seph[i].acc[2];

    }
    for(i=0; i<conv->raw.nav.nemax; i++){  //time not implemented
        int e;

        navigmsg.peph[i].index=conv->raw.nav.peph[i].index;      
        for(e=0; e<MAXSAT; e++){
           // for(d=0; d<4; d++){
                navigmsg.peph[i].pos[e]=conv->raw.nav.peph[i].pos[e][1];  // How do I represent double array in ROS msg format
                navigmsg.peph[i].std[e]=conv->raw.nav.peph[i].std[e][1];
           // }
        }
    }

    for(i=0; i<conv->raw.nav.ncmax; i++){
        navigmsg.pclk[i].index=conv->raw.nav.pclk[i].index;
        int e;
        for(e=0; e<MAXSAT; e++){
                navigmsg.pclk[i].clk[e]=conv->raw.nav.pclk[i].clk[e][1];
                navigmsg.pclk[i].std[e]=conv->raw.nav.pclk[i].std[e][1];
        }
    }

    for(i=0; i<conv->raw.nav.namax; i++){    //time not implemented gtime_t
        navigmsg.alm[i].sat=conv->raw.nav.alm[i].sat;
        navigmsg.alm[i].svh=conv->raw.nav.alm[i].svh;
        navigmsg.alm[i].svconf=conv->raw.nav.alm[i].svconf;
        navigmsg.alm[i].week=conv->raw.nav.alm[i].week;
        navigmsg.alm[i].A=conv->raw.nav.alm[i].A;
        navigmsg.alm[i].e=conv->raw.nav.alm[i].e;
        navigmsg.alm[i].i0=conv->raw.nav.alm[i].i0;
        navigmsg.alm[i].OMG0=conv->raw.nav.alm[i].OMG0;
        navigmsg.alm[i].omg=conv->raw.nav.alm[i].omg;
        navigmsg.alm[i].M0=conv->raw.nav.alm[i].M0;
        navigmsg.alm[i].toas=conv->raw.nav.alm[i].toas;
        navigmsg.alm[i].f0=conv->raw.nav.alm[i].f0;
        navigmsg.alm[i].f1=conv->raw.nav.alm[i].f1;
    }

    for(i=0; i<conv->raw.nav.ntmax; i++){   //time not implemented with gtime_t
        navigmsg.tec[i].rb=conv->raw.nav.tec[i].rb;
        int f;
        for(f=0; f<3; f++){
            navigmsg.tec[i].lats[f]=conv->raw.nav.tec[i].lats[f];
            navigmsg.tec[i].lons[f]=conv->raw.nav.tec[i].lons[f];
            navigmsg.tec[i].hgts[f]=conv->raw.nav.tec[i].hgts[f];
            navigmsg.tec[i].ndata[f]=conv->raw.nav.tec[i].ndata[f];
            //not sure about the following 2 whether their size is 3
            navigmsg.tec[i].data[f]=conv->raw.nav.tec[i].data[f];
            navigmsg.tec[i].rms[f]=conv->raw.nav.tec[i].rms[f];
        }

    }

    for(i=0; i<conv->raw.nav.nnmax; i++){
        navigmsg.stec[i].n=conv->raw.nav.stec[i].n;
        navigmsg.stec[i].nmax=conv->raw.nav.stec[i].nmax;
        navigmsg.stec[i].pos[1]=conv->raw.nav.stec[i].pos[1];
        navigmsg.stec[i].pos[2]=conv->raw.nav.stec[i].pos[2];
        int g,h; 
        for(g=0; g<MAXSAT; g++){
            navigmsg.stec[i].index[g]=conv->raw.nav.stec[i].index[g];
        }

        for(h=0; h<conv->raw.nav.stec[i].nmax; h++){
            navigmsg.stec[i].data[h].sat=conv->raw.nav.stec[i].data[h].sat;
            navigmsg.stec[i].data[h].slip=conv->raw.nav.stec[i].data[h].slip;
            navigmsg.stec[i].data[h].iono=conv->raw.nav.stec[i].data[h].iono;
            navigmsg.stec[i].data[h].rate=conv->raw.nav.stec[i].data[h].rate;
            navigmsg.stec[i].data[h].rms=conv->raw.nav.stec[i].data[h].rms;
        }
    }
    
    
    for(i=0; i<conv->raw.nav.erp.nmax;i++){
        navigmsg.erp.data[i].mjd =conv->raw.nav.erp.data[i].mjd;
        navigmsg.erp.data[i].xp =conv->raw.nav.erp.data[i].xp;
        navigmsg.erp.data[i].yp =conv->raw.nav.erp.data[i].yp;
        navigmsg.erp.data[i].xpr =conv->raw.nav.erp.data[i].xpr;
        navigmsg.erp.data[i].ypr =conv->raw.nav.erp.data[i].ypr;
        navigmsg.erp.data[i].ut1_utc =conv->raw.nav.erp.data[i].ut1_utc;
        navigmsg.erp.data[i].lod =conv->raw.nav.erp.data[i].lod;
    }

    for(i=0; i<MAXSAT; i++){
        navigmsg.pcvs[i].sat=conv->raw.nav.pcvs[i].sat;   //you have to add some component here
    }

    navigmsg.sbssat.iodp=conv->raw.nav.sbssat.iodp;
    navigmsg.sbssat.nsat=conv->raw.nav.sbssat.nsat;
    navigmsg.sbssat.tlat=conv->raw.nav.sbssat.tlat;      //  you have to add some component here



    for(i=0; i<(MAXBAND+1); i++){   //because MAXBAND=10
        navigmsg.sbsion[i].iodi=conv->raw.nav.sbsion[i].iodi;
        navigmsg.sbsion[i].nigp=conv->raw.nav.sbsion[i].nigp;
        
        for(p=0; p<MAXNIGP; p++){ //did not implement time
            navigmsg.sbsion[i].igp[p].lat=conv->raw.nav.sbsion[i].igp[p].lat;
            navigmsg.sbsion[i].igp[p].lon=conv->raw.nav.sbsion[i].igp[p].lon;
            navigmsg.sbsion[i].igp[p].give=conv->raw.nav.sbsion[i].igp[p].give;
            navigmsg.sbsion[i].igp[p].delay=conv->raw.nav.sbsion[i].igp[p].delay;
        }
    }
    
    for(i=0; i<MAXSAT; i++){
        navigmsg.dgps[i].prc=conv->raw.nav.dgps[i].prc;
        navigmsg.dgps[i].rrc=conv->raw.nav.dgps[i].rrc;
        navigmsg.dgps[i].iod=conv->raw.nav.dgps[i].iod;
        navigmsg.dgps[i].udre=conv->raw.nav.dgps[i].udre;
    }
    
    for(i=0; i<MAXSAT; i++){
        int m, n;
        for(m=0; m<5; m++){
            navigmsg.ssr[i].udi[m]=conv->raw.nav.ssr[i].udi[m];
            navigmsg.ssr[i].iod[m]=conv->raw.nav.ssr[i].iod[m];
        }

        for(n=0; n<3; n++){
            navigmsg.ssr[i].deph[n]=conv->raw.nav.ssr[i].deph[n];
            navigmsg.ssr[i].ddeph[n]=conv->raw.nav.ssr[i].ddeph[n];
            navigmsg.ssr[i].dclk[n]=conv->raw.nav.ssr[i].dclk[n];
        }

        navigmsg.ssr[i].iode=conv->raw.nav.ssr[i].iode;
        navigmsg.ssr[i].ura=conv->raw.nav.ssr[i].ura;
        navigmsg.ssr[i].refd=conv->raw.nav.ssr[i].refd;
        navigmsg.ssr[i].hrclk=conv->raw.nav.ssr[i].hrclk;
        navigmsg.ssr[i].update=conv->raw.nav.ssr[i].update;

        int b;
        for(b=0;b<MAXCODE; b++){
            navigmsg.ssr[i].cbias[b]=conv->raw.nav.ssr[i].cbias[b];
        }
    }
    
    for(i=0; i<MAXSAT; i++){   //time not integrated yet

        navigmsg.lexeph[i].af0=conv->raw.nav.lexeph[i].af0;
        navigmsg.lexeph[i].af1=conv->raw.nav.lexeph[i].af1;
        navigmsg.lexeph[i].tgd=conv->raw.nav.lexeph[i].tgd;
        navigmsg.lexeph[i].sat=conv->raw.nav.lexeph[i].sat;
        navigmsg.lexeph[i].health=conv->raw.nav.lexeph[i].health;
        navigmsg.lexeph[i].ura=conv->raw.nav.lexeph[i].ura;
        navigmsg.lexeph[i].pos.x=conv->raw.nav.lexeph[i].pos[0];
        navigmsg.lexeph[i].pos.y=conv->raw.nav.lexeph[i].pos[1];
        navigmsg.lexeph[i].pos.z=conv->raw.nav.lexeph[i].pos[2];

        navigmsg.lexeph[i].vel.x=conv->raw.nav.lexeph[i].vel[0];
        navigmsg.lexeph[i].vel.y=conv->raw.nav.lexeph[i].vel[1];
        navigmsg.lexeph[i].vel.z=conv->raw.nav.lexeph[i].vel[2];

        navigmsg.lexeph[i].acc.x=conv->raw.nav.lexeph[i].acc[0];
        navigmsg.lexeph[i].acc.y=conv->raw.nav.lexeph[i].acc[1];
        navigmsg.lexeph[i].acc.z=conv->raw.nav.lexeph[i].acc[2];

        navigmsg.lexeph[i].jerk.x=conv->raw.nav.lexeph[i].jerk[0];
        navigmsg.lexeph[i].jerk.y=conv->raw.nav.lexeph[i].jerk[1];
        navigmsg.lexeph[i].jerk.z=conv->raw.nav.lexeph[i].jerk[2];

        int o;
        for(o=0; o<8;o++){
            navigmsg.lexeph[i].isc[o]=conv->raw.nav.lexeph[i].isc[o];
        }
    }

    navigmsg.lexion.tspan=conv->raw.nav.lexion.tspan;
    navigmsg.lexion.pos0[0]=conv->raw.nav.lexion.pos0[0];
    navigmsg.lexion.pos0[1]=conv->raw.nav.lexion.pos0[1];
   /*   Get this checked
    navigmsg.lexion.coef[0]=conv->raw.nav.lexion.coef[1][1];
    navigmsg.lexion.coef[1]=conv->raw.nav.lexion.coef[1][2];
    navigmsg.lexion.coef[2]=conv->raw.nav.lexion.coef[2][1];
    navigmsg.lexion.coef[3]=conv->raw.nav.lexion.coef[2][2];
    navigmsg.lexion.coef[4]=conv->raw.nav.lexion.coef[3][1];
    navigmsg.lexion.coef[5]=conv->raw.nav.lexion.coef[3][2];
*/
    
    for(i=0; i<MAXSAT; i++){
        navigmsg.lamcwl[i].L1=conv->raw.nav.lam[i][0];
        navigmsg.lamcwl[i].L2=conv->raw.nav.lam[i][1];
        navigmsg.lamcwl[i].L5=conv->raw.nav.lam[i][2];
        navigmsg.lamcwl[i].L6=conv->raw.nav.lam[i][3];
        navigmsg.lamcwl[i].L7=conv->raw.nav.lam[i][4];
        navigmsg.lamcwl[i].L8=conv->raw.nav.lam[i][5];
    }
    
    for(i=0; i<MAXSAT; i++){
        navigmsg.codebias[i].p1_p2=conv->raw.nav.cbias[i][0];
        navigmsg.codebias[i].p1_c1=conv->raw.nav.cbias[i][1];
        navigmsg.codebias[i].p2_c2=conv->raw.nav.cbias[i][2];
    }
    
    for(j=0; j<4; j++){
        navigmsg.utc_gps[j]=conv->raw.nav.utc_gps[j];
        navigmsg.utc_glo[j]=conv->raw.nav.utc_glo[j];
        navigmsg.utc_gal[j]=conv->raw.nav.utc_gal[j];
        navigmsg.utc_qzs[j]=conv->raw.nav.utc_qzs[j];
        navigmsg.utc_cmp[j]=conv->raw.nav.utc_cmp[j];
        navigmsg.utc_sbs[j]=conv->raw.nav.utc_sbs[j];
        navigmsg.ion_gal[j]=conv->raw.nav.ion_gal[j];
    }
    for(j=0; j<8; j++){ 
        navigmsg.ion_gps[j]=conv->raw.nav.ion_gps[j];
        navigmsg.ion_qzs[j]=conv->raw.nav.ion_qzs[j];
        navigmsg.ion_cmp[j]=conv->raw.nav.ion_cmp[j];
    }
    navigmsg.leaps=conv->raw.nav.leaps;

}

// convert stream -------------------------------------------------------------------------------
static void raw2ros(ros::Publisher& pub_navigation_msg, ros::Publisher& pub_observation_msg, strconv_t *conv, unsigned char *buff, int n)
{
    int ret,k;

    rtklib::Navigation navmsg;
    rtklib::Observation obsmsg;
    rtklib::ObservationData obsdata;

    for (k=0;k<n;k++) {
        // input rtcm 2 messages 
        if (conv->itype==STRFMT_RTCM2) {
            continue;
        }
        // input rtcm 3 messages
        else if (conv->itype==STRFMT_RTCM3) {
            continue;
        }
        
        // input receiver raw messages
        else {
            ret=input_raw(&conv->raw,conv->itype,buff[k]);

			if(ret==1){
                obsfunc(obsmsg,obsdata,conv);
			}

            if(ret==2){
                navmsgfunc(navmsg, conv);
            }

            pub_navigation_msg.publish(navmsg);
            pub_observation_msg.publish(obsmsg);
        }
    }
}


int main(int argc, char** argv){

    ros::NodeHandle nh;

    ros::Publisher pub_navigation_msg = nh.advertise<rtklib::Navigation>("nav_topic", 10);    
    ros::Publisher pub_observation_msg = nh.advertise<rtklib::Observation>("obs_topic", 10);
    ros::Rate r(1);
    strconv_t *conv;
    unsigned char *buff; 
    int n;

    while (ros::ok()){

        raw2ros(pub_navigation_msg, pub_observation_msg, conv, buff, n);
        ros::spinOnce();
    }

return 0;
}