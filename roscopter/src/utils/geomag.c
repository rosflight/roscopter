
// March 2020: Made to work as a small and easy to use library for GNU/Linux C programs by M. Wirth
// July 2024: Trimmed for use in ROScopter by Ian Reid.
// Original notes provided below
/* PROGRAM MAGPOINT (GEOMAG DRIVER) */
/************************************************************************

     Contact Information

     Software and Model Support
     	National Geophysical Data Center
     	NOAA EGC/2
     	325 Broadway
     	Boulder, CO 80303 USA
		Attn: Manoj Nair or Stefan Maus
		Phone:  (303) 497-4642 or -6522
		Email:  Manoj.C.Nair@Noaa.gov or Stefan.Maus@noaa.gov
		Web: http://www.ngdc.noaa.gov/geomag/WMM/

	 Sponsoring Government Agency
	   National Geospatial-Intelligence Agency
    	   PRG / CSAT, M.S. L-41
    	   3838 Vogel Road
    	   Arnold, MO 63010
    	   Attn: Craig Rollins
    	   Phone:  (314) 263-4186
    	   Email:  Craig.M.Rollins@Nga.Mil

      Original Program By:
        Dr. John Quinn
        FLEET PRODUCTS DIVISION, CODE N342
        NAVAL OCEANOGRAPHIC OFFICE (NAVOCEANO)
        STENNIS SPACE CENTER (SSC), MS 39522-5001

		3/25/05 Version 2.0 Stefan Maus corrected 2 bugs:
         - use %c instead of %s for character read
		 - help text: positive inclination is downward
		1/29/2010 Version 3.0 Manoj Nair
		Converted floating variables from single precision to double
		Changed : height above AMSL (WGS84) to Height above WGS84 Ellipsoid
		Removed the NaN forcing at the geographic poles
		A new function "my_isnan" for improved portablility

    --------------------------------------------------------------

    The following came as the introduction to this file. This has been edited by 
    Ian Reid of the BYU MAGICC Lab to remove the interactive mode.

    Note: Every 5 years the magnetic coeffeicients need to be updated.

    --------------------------------------------------------------
    
    Welcome to the World Magnetic Model (WMM) %4.0lf C-Program\n\n
               --- Version 3.0, January 2010 ---\n\n
    This program estimates the strength and direction of 
    Earth's main magnetic field for a given point/area.
    Enter h for help and contact information or c to continue.
    The World Magnetic Model (WMM) for %7.2lf
    is a model of Earth's main magnetic field.  The WMM
    is recomputed every five (5) years, in years divisible by 
    five (i.e. 2010, 2015).  See the contact information below
    to obtain more information on the WMM and associated software.
    
    Input required is the location in geodetic latitude and
    longitude (positive for northern latitudes and eastern 
    longitudes), geodetic altitude in meters, and the date of 
    interest in years.

    The program computes the estimated magnetic Declination
    (D) which is sometimes called MAGVAR, Inclination (I), Total
    Intensity (F or TI), Horizontal Intensity (H or HI), Vertical
    Intensity (Z), and Grid Variation (GV). Declination and Grid
    Variation are measured in units of degrees and are considered
    positive when east or north.  Inclination is measured in units
    of degrees and is considered positive when pointing down (into
    the Earth).  The WMM is reference to the WGS-84 ellipsoid and
    is valid for 5 years after the base epoch.

    It is very important to note that a  degree and  order 12 model,
    such as WMM, describes only the long  wavelength spatial magnetic 
    fluctuations due to  Earth's core.  Not included in the WMM series
    models are intermediate and short wavelength spatial fluctuations 
    that originate in Earth's mantle and crust. Consequently, isolated
    angular errors at various  positions on the surface (primarily over
    land, incontinental margins and  over oceanic seamounts, ridges and
    trenches) of several degrees may be expected.  Also not included in
    the model are temporal fluctuations of magnetospheric and ionospheric
    origin. On the days during and immediately following magnetic storms,
    temporal fluctuations can cause substantial deviations of the geomagnetic
    field  from model  values.  If the required  declination accuracy  is
    more stringent than the WMM  series of models provide, the user is
    advised to request special (regional or local) surveys be performed
    and models prepared. Please make requests of this nature to the
    National Geospatial-Intelligence Agency (NGA) at the address below.


*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define NaN log(-1.0)

static char **wmm_lines;
static char *wmm_string;
static int wmm_index;
static int maxdeg;
static double epochlowlim,epochuplim,epoch;
char decd[7], dipd[7],modl[20];

static char* goodbye = "\n -- End of WMM Point Calculation Program -- \n\n";

static int my_isnan(double d)
{
    return (d != d);              /* IEEE: only NaN is not equal to itself */
}

static int geomag_E0_init(int *maxdeg);
static char geomag_introduction(double epochlowlim);

int geomag_destroy() {
    free(wmm_lines);
    wmm_lines = NULL;
    free(wmm_string);
    wmm_string = NULL;
    return 0;
}

int geomag_init()
{

    // hard code WMM2020, this can easily enough be replaced
    // i don't like having to read a file, better to have the data in the source code.
    // when replacing this, make sure to add \n\ at the end of each line to the text from the WMM.COF
    // also make sure the indentation stays as it is ... some code depends on that.
    // UPDATE THIS EVERY 5 YEARS 2025, 2030, 2035, 2040...

    wmm_string = strdup("\
    2020.0            WMM-2020        12/10/2019\n\
  1  0  -29404.5       0.0        6.7        0.0\n\
  1  1   -1450.7    4652.9        7.7      -25.1\n\
  2  0   -2500.0       0.0      -11.5        0.0\n\
  2  1    2982.0   -2991.6       -7.1      -30.2\n\
  2  2    1676.8    -734.8       -2.2      -23.9\n\
  3  0    1363.9       0.0        2.8        0.0\n\
  3  1   -2381.0     -82.2       -6.2        5.7\n\
  3  2    1236.2     241.8        3.4       -1.0\n\
  3  3     525.7    -542.9      -12.2        1.1\n\
  4  0     903.1       0.0       -1.1        0.0\n\
  4  1     809.4     282.0       -1.6        0.2\n\
  4  2      86.2    -158.4       -6.0        6.9\n\
  4  3    -309.4     199.8        5.4        3.7\n\
  4  4      47.9    -350.1       -5.5       -5.6\n\
  5  0    -234.4       0.0       -0.3        0.0\n\
  5  1     363.1      47.7        0.6        0.1\n\
  5  2     187.8     208.4       -0.7        2.5\n\
  5  3    -140.7    -121.3        0.1       -0.9\n\
  5  4    -151.2      32.2        1.2        3.0\n\
  5  5      13.7      99.1        1.0        0.5\n\
  6  0      65.9       0.0       -0.6        0.0\n\
  6  1      65.6     -19.1       -0.4        0.1\n\
  6  2      73.0      25.0        0.5       -1.8\n\
  6  3    -121.5      52.7        1.4       -1.4\n\
  6  4     -36.2     -64.4       -1.4        0.9\n\
  6  5      13.5       9.0       -0.0        0.1\n\
  6  6     -64.7      68.1        0.8        1.0\n\
  7  0      80.6       0.0       -0.1        0.0\n\
  7  1     -76.8     -51.4       -0.3        0.5\n\
  7  2      -8.3     -16.8       -0.1        0.6\n\
  7  3      56.5       2.3        0.7       -0.7\n\
  7  4      15.8      23.5        0.2       -0.2\n\
  7  5       6.4      -2.2       -0.5       -1.2\n\
  7  6      -7.2     -27.2       -0.8        0.2\n\
  7  7       9.8      -1.9        1.0        0.3\n\
  8  0      23.6       0.0       -0.1        0.0\n\
  8  1       9.8       8.4        0.1       -0.3\n\
  8  2     -17.5     -15.3       -0.1        0.7\n\
  8  3      -0.4      12.8        0.5       -0.2\n\
  8  4     -21.1     -11.8       -0.1        0.5\n\
  8  5      15.3      14.9        0.4       -0.3\n\
  8  6      13.7       3.6        0.5       -0.5\n\
  8  7     -16.5      -6.9        0.0        0.4\n\
  8  8      -0.3       2.8        0.4        0.1\n\
  9  0       5.0       0.0       -0.1        0.0\n\
  9  1       8.2     -23.3       -0.2       -0.3\n\
  9  2       2.9      11.1       -0.0        0.2\n\
  9  3      -1.4       9.8        0.4       -0.4\n\
  9  4      -1.1      -5.1       -0.3        0.4\n\
  9  5     -13.3      -6.2       -0.0        0.1\n\
  9  6       1.1       7.8        0.3       -0.0\n\
  9  7       8.9       0.4       -0.0       -0.2\n\
  9  8      -9.3      -1.5       -0.0        0.5\n\
  9  9     -11.9       9.7       -0.4        0.2\n\
 10  0      -1.9       0.0        0.0        0.0\n\
 10  1      -6.2       3.4       -0.0       -0.0\n\
 10  2      -0.1      -0.2       -0.0        0.1\n\
 10  3       1.7       3.5        0.2       -0.3\n\
 10  4      -0.9       4.8       -0.1        0.1\n\
 10  5       0.6      -8.6       -0.2       -0.2\n\
 10  6      -0.9      -0.1       -0.0        0.1\n\
 10  7       1.9      -4.2       -0.1       -0.0\n\
 10  8       1.4      -3.4       -0.2       -0.1\n\
 10  9      -2.4      -0.1       -0.1        0.2\n\
 10 10      -3.9      -8.8       -0.0       -0.0\n\
 11  0       3.0       0.0       -0.0        0.0\n\
 11  1      -1.4      -0.0       -0.1       -0.0\n\
 11  2      -2.5       2.6       -0.0        0.1\n\
 11  3       2.4      -0.5        0.0        0.0\n\
 11  4      -0.9      -0.4       -0.0        0.2\n\
 11  5       0.3       0.6       -0.1       -0.0\n\
 11  6      -0.7      -0.2        0.0        0.0\n\
 11  7      -0.1      -1.7       -0.0        0.1\n\
 11  8       1.4      -1.6       -0.1       -0.0\n\
 11  9      -0.6      -3.0       -0.1       -0.1\n\
 11 10       0.2      -2.0       -0.1        0.0\n\
 11 11       3.1      -2.6       -0.1       -0.0\n\
 12  0      -2.0       0.0        0.0        0.0\n\
 12  1      -0.1      -1.2       -0.0       -0.0\n\
 12  2       0.5       0.5       -0.0        0.0\n\
 12  3       1.3       1.3        0.0       -0.1\n\
 12  4      -1.2      -1.8       -0.0        0.1\n\
 12  5       0.7       0.1       -0.0       -0.0\n\
 12  6       0.3       0.7        0.0        0.0\n\
 12  7       0.5      -0.1       -0.0       -0.0\n\
 12  8      -0.2       0.6        0.0        0.1\n\
 12  9      -0.5       0.2       -0.0       -0.0\n\
 12 10       0.1      -0.9       -0.0       -0.0\n\
 12 11      -1.1      -0.0       -0.0        0.0\n\
 12 12      -0.3       0.5       -0.1       -0.1\n\
999999999999999999999999999999999999999999999999\n\
999999999999999999999999999999999999999999999999\n\
");

    wmm_lines = malloc(sizeof(char*) * 256);
    wmm_index = 0;
    wmm_lines[wmm_index] = strtok(wmm_string, "\n");
    while (wmm_lines[wmm_index]) {
        wmm_index++;
        wmm_lines[wmm_index] = strtok(NULL, "\n");
    }


    if (wmm_lines[0] == NULL || sscanf(wmm_lines[0],"%lf%s",&epochlowlim,modl) < 2)
    {
        fprintf(stderr, "Invalid header in model wmm_string in geomag.c\n");
        return -1;
    }

    /* INITIALIZE GEOMAG ROUTINE */

    maxdeg = 12;
    return geomag_E0_init(&maxdeg);
}

/*************************************************************************/

static int E0000(int IENTRY, int *maxdeg, double alt, double glat, double glon, double time, double *dec, double *dip, double *ti, double *gv)
{
    static int maxord,i,icomp,n,m,j,D1,D2,D3,D4;
    static double c[13][13],cd[13][13],tc[13][13],dp[13][13],snorm[169],
    sp[13],cp[13],fn[13],fm[13],pp[13],k[13][13],pi,dtr,a,b,re,
    a2,b2,c2,a4,b4,c4,gnm,hnm,dgnm,dhnm,flnmj,
    dt,rlon,rlat,srlon,srlat,crlon,crlat,srlat2,
    crlat2,q,q1,q2,ct,st,r2,r,d,ca,sa,aor,ar,br,bt,bp,bpp,
    par,temp1,temp2,parp,bx,by,bz,bh;
    static char model[20], c_new[5];
    static double *p = snorm;

    switch(IENTRY){case 0: goto INIT; case 1: goto CALC;}

INIT:

    /* INITIALIZE CONSTANTS */
    maxord = *maxdeg;
    sp[0] = 0.0;
    cp[0] = *p = pp[0] = 1.0;
    dp[0][0] = 0.0;
    a = 6378.137;
    b = 6356.7523142;
    re = 6371.2;
    a2 = a*a;
    b2 = b*b;
    c2 = a2-b2;
    a4 = a2*a2;
    b4 = b2*b2;
    c4 = a4 - b4;

    /* READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS */
    c[0][0] = 0.0;
    cd[0][0] = 0.0;

    wmm_index = 0;
    if (wmm_lines[wmm_index] == NULL || sscanf(wmm_lines[wmm_index],"%lf%s",&epoch,model) < 2) 
    {
        fprintf(stderr, "Invalid header in model wmm_string in geomag.c\n");
        return -1;
    }

S3:
    wmm_index++;
    if (wmm_lines[wmm_index] == NULL) goto S4;

    /* CHECK FOR LAST LINE IN FILE */
    for (i=0; i<4 && (wmm_lines[wmm_index][i] != '\0'); i++)
    {
        c_new[i] = wmm_lines[wmm_index][i];
        c_new[i+1] = '\0';
    }
    icomp = strcmp("9999", c_new);
    if (icomp == 0) goto S4;
    /* END OF FILE NOT ENCOUNTERED, GET VALUES */
    sscanf(wmm_lines[wmm_index], "%d%d%lf%lf%lf%lf",&n,&m,&gnm,&hnm,&dgnm,&dhnm);

    if (n > maxord) goto S4;
    if (m > n || m < 0.0) 
    {
        fprintf(stderr, "%d\n", wmm_index);
        fprintf(stderr, "Corrupt record in model wmm_string in geomag.c\n");
        return -1;
    }

    if (m <= n)
    {
        c[m][n] = gnm;
        cd[m][n] = dgnm;
        if (m != 0)
        {
            c[n][m-1] = hnm;
            cd[n][m-1] = dhnm;
        }
    }
    goto S3;

    /* CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED */
S4:
    *snorm = 1.0;
    fm[0] = 0.0;
    for (n=1; n<=maxord; n++)
    {
        *(snorm+n) = *(snorm+n-1)*(double)(2*n-1)/(double)n;
        j = 2;
        for (m=0,D1=1,D2=(n-m+D1)/D1; D2>0; D2--,m+=D1)
        {
            k[m][n] = (double)(((n-1)*(n-1))-(m*m))/(double)((2*n-1)*(2*n-3));
            if (m > 0)
            {
                flnmj = (double)((n-m+1)*j)/(double)(n+m);
                *(snorm+n+m*13) = *(snorm+n+(m-1)*13)*sqrt(flnmj);
                j = 1;
                c[n][m-1] = *(snorm+n+m*13)*c[n][m-1];
                cd[n][m-1] = *(snorm+n+m*13)*cd[n][m-1];
            }
            c[m][n] = *(snorm+n+m*13)*c[m][n];
            cd[m][n] = *(snorm+n+m*13)*cd[m][n];
        }
        fn[n] = (double)(n+1);
        fm[n] = (double)n;
    }
    k[1][1] = 0.0;

    return 0;

    /*************************************************************************/

CALC:

    dt = time - epoch;

    pi = 3.14159265359;
    dtr = pi/180.0;
    rlon = glon*dtr;
    rlat = glat*dtr;
    srlon = sin(rlon);
    srlat = sin(rlat);
    crlon = cos(rlon);
    crlat = cos(rlat);
    srlat2 = srlat*srlat;
    crlat2 = crlat*crlat;
    sp[1] = srlon;
    cp[1] = crlon;

    /* CONVERT FROM GEODETIC COORDS. TO SPHERICAL COORDS. */
    q = sqrt(a2-c2*srlat2);
    q1 = alt*q;
    q2 = ((q1+a2)/(q1+b2))*((q1+a2)/(q1+b2));
    ct = srlat/sqrt(q2*crlat2+srlat2);
    st = sqrt(1.0-(ct*ct));
    r2 = (alt*alt)+2.0*q1+(a4-c4*srlat2)/(q*q);
    r = sqrt(r2);
    d = sqrt(a2*crlat2+b2*srlat2);
    ca = (alt+d)/r;
    sa = c2*crlat*srlat/(r*d);

    for (m=2; m<=maxord; m++)
    {
        sp[m] = sp[1]*cp[m-1]+cp[1]*sp[m-1];
        cp[m] = cp[1]*cp[m-1]-sp[1]*sp[m-1];
    }

    aor = re/r;
    ar = aor*aor;
    br = bt = bp = bpp = 0.0;
    for (n=1; n<=maxord; n++)
    {
        ar = ar*aor;
        for (m=0,D3=1,D4=(n+m+D3)/D3; D4>0; D4--,m+=D3)
        {
            /*
               COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS
               AND DERIVATIVES VIA RECURSION RELATIONS
               */
            if (n == m)
            {
                *(p+n+m*13) = st**(p+n-1+(m-1)*13);
                dp[m][n] = st*dp[m-1][n-1]+ct**(p+n-1+(m-1)*13);
                goto S50;
            }
            if (n == 1 && m == 0)
            {
                *(p+n+m*13) = ct**(p+n-1+m*13);
                dp[m][n] = ct*dp[m][n-1]-st**(p+n-1+m*13);
                goto S50;
            }
            if (n > 1 && n != m)
            {
                if (m > n-2) *(p+n-2+m*13) = 0.0;
                if (m > n-2) dp[m][n-2] = 0.0;
                *(p+n+m*13) = ct**(p+n-1+m*13)-k[m][n]**(p+n-2+m*13);
                dp[m][n] = ct*dp[m][n-1] - st**(p+n-1+m*13)-k[m][n]*dp[m][n-2];
            }
S50:
            /*
               TIME ADJUST THE GAUSS COEFFICIENTS
               */
            tc[m][n] = c[m][n]+dt*cd[m][n];
            if (m != 0) tc[n][m-1] = c[n][m-1]+dt*cd[n][m-1];
            /*
               ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
               */
            par = ar**(p+n+m*13);
            if (m == 0)
            {
                temp1 = tc[m][n]*cp[m];
                temp2 = tc[m][n]*sp[m];
            }
            else
            {
                temp1 = tc[m][n]*cp[m]+tc[n][m-1]*sp[m];
                temp2 = tc[m][n]*sp[m]-tc[n][m-1]*cp[m];
            }
            bt = bt-ar*temp1*dp[m][n];
            bp += (fm[m]*temp2*par);
            br += (fn[n]*temp1*par);
            /*
               SPECIAL CASE:  NORTH/SOUTH GEOGRAPHIC POLES
               */
            if (st == 0.0 && m == 1)
            {
                if (n == 1) pp[n] = pp[n-1];
                else pp[n] = ct*pp[n-1]-k[m][n]*pp[n-2];
                parp = ar*pp[n];
                bpp += (fm[m]*temp2*parp);
            }
        }
    }
    if (st == 0.0) bp = bpp;
    else bp /= st;
    /*
       ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO
       GEODETIC COORDINATES
       */
    bx = -bt*ca-br*sa;
    by = bp;
    bz = bt*sa-br*ca;
    /*
       COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND
       TOTAL INTENSITY (TI)
       */
    bh = sqrt((bx*bx)+(by*by));
    *ti = sqrt((bh*bh)+(bz*bz));
    *dec = atan2(by,bx)/dtr;
    *dip = atan2(bz,bh)/dtr;
    /*
       COMPUTE MAGNETIC GRID VARIATION IF THE CURRENT
       GEODETIC POSITION IS IN THE ARCTIC OR ANTARCTIC
       (I.E. GLAT > +55 DEGREES OR GLAT < -55 DEGREES)

       OTHERWISE, SET MAGNETIC GRID VARIATION TO -999.0
       */
    *gv = -999.0;
    if (fabs(glat) >= 55.)
    {
        if (glat > 0.0 && glon >= 0.0) *gv = *dec-glon;
        if (glat > 0.0 && glon < 0.0) *gv = *dec+fabs(glon);
        if (glat < 0.0 && glon >= 0.0) *gv = *dec+glon;
        if (glat < 0.0 && glon < 0.0) *gv = *dec-fabs(glon);
        if (*gv > +180.0) *gv -= 360.0;
        if (*gv < -180.0) *gv += 360.0;
    }
    return 0;
}

/*************************************************************************/

static int geomag_E0_init(int *maxdeg)
{
    return E0000(0,maxdeg,0.0,0.0,0.0,0.0,NULL,NULL,NULL,NULL);
}

/*************************************************************************/

int geomag_calc(double alt, double glat, double glon, double time, double *dec, double *dip, double *ti, double *gv)
{
    return E0000(1,NULL,alt,glat,glon,time,dec,dip,ti,gv);
}

