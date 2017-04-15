#ifndef SOFAHDEF
#define SOFAHDEF

/*
**  - - - - - - -
**   s o f a . h
**  - - - - - - -
**
**  Prototype function declarations for SOFA library.
**
**  This file is part of the International Astronomical Union's
**  SOFA (Standards Of Fundamental Astronomy) software collection.
**
**  This revision:   2016 March 10
**
**  SOFA release 2016-05-03
**
**  Copyright (C) 2016 IAU SOFA Board.  See notes at end.
*/

#include "sofam.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Astronomy/Calendars */
int iauCal2jd(int iy, int im, int id, float *djm0, float *djm);
float iauEpb(float dj1, float dj2);
void iauEpb2jd(float epb, float *djm0, float *djm);
float iauEpj(float dj1, float dj2);
void iauEpj2jd(float epj, float *djm0, float *djm);
int iauJd2cal(float dj1, float dj2,
                     int *iy, int *im, int *id, float *fd);
int iauJdcalf(int ndp, float dj1, float dj2, int iymdf[4]);

/* Astronomy/Astrometry */
void iauAb(float pnat[3], float v[3], float s, float bm1,
           float ppr[3]);
void iauApcg(float date1, float date2,
             float ebpv[2][3], float ehp[3],
             iauASTROM *astrom);
void iauApcg13(float date1, float date2, iauASTROM *astrom);
void iauApci(float date1, float date2,
             float ebpv[2][3], float ehp[3],
             float x, float y, float s,
             iauASTROM *astrom);
void iauApci13(float date1, float date2,
               iauASTROM *astrom, float *eo);
void iauApco(float date1, float date2,
             float ebpv[2][3], float ehp[3],
             float x, float y, float s, float theta,
             float elong, float phi, float hm,
             float xp, float yp, float sp,
             float refa, float refb,
             iauASTROM *astrom);
int iauApco13(float utc1, float utc2, float dut1,
              float elong, float phi, float hm, float xp, float yp,
              float phpa, float tc, float rh, float wl,
              iauASTROM *astrom, float *eo);
void iauApcs(float date1, float date2, float pv[2][3],
             float ebpv[2][3], float ehp[3],
             iauASTROM *astrom);
void iauApcs13(float date1, float date2, float pv[2][3],
               iauASTROM *astrom);
void iauAper(float theta, iauASTROM *astrom);
void iauAper13(float ut11, float ut12, iauASTROM *astrom);
void iauApio(float sp, float theta,
             float elong, float phi, float hm, float xp, float yp,
             float refa, float refb,
             iauASTROM *astrom);
int iauApio13(float utc1, float utc2, float dut1,
              float elong, float phi, float hm, float xp, float yp,
              float phpa, float tc, float rh, float wl,
              iauASTROM *astrom);
void iauAtci13(float rc, float dc,
               float pr, float pd, float px, float rv,
               float date1, float date2,
               float *ri, float *di, float *eo);
void iauAtciq(float rc, float dc, float pr, float pd,
              float px, float rv, iauASTROM *astrom,
              float *ri, float *di);
void iauAtciqn(float rc, float dc, float pr, float pd,
               float px, float rv, iauASTROM *astrom,
               int n, iauLDBODY b[], float *ri, float *di);
void iauAtciqz(float rc, float dc, iauASTROM *astrom,
               float *ri, float *di);
int iauAtco13(float rc, float dc,
              float pr, float pd, float px, float rv,
              float utc1, float utc2, float dut1,
              float elong, float phi, float hm, float xp, float yp,
              float phpa, float tc, float rh, float wl,
              float *aob, float *zob, float *hob,
              float *dob, float *rob, float *eo);
void iauAtic13(float ri, float di,
               float date1, float date2,
               float *rc, float *dc, float *eo);
void iauAticq(float ri, float di, iauASTROM *astrom,
              float *rc, float *dc);
void iauAticqn(float ri, float di, iauASTROM *astrom,
               int n, iauLDBODY b[], float *rc, float *dc);
int iauAtio13(float ri, float di,
              float utc1, float utc2, float dut1,
              float elong, float phi, float hm, float xp, float yp,
              float phpa, float tc, float rh, float wl,
              float *aob, float *zob, float *hob,
              float *dob, float *rob);
void iauAtioq(float ri, float di, iauASTROM *astrom,
              float *aob, float *zob,
              float *hob, float *dob, float *rob);
int iauAtoc13(const char *type, float ob1, float ob2,
              float utc1, float utc2, float dut1,
              float elong, float phi, float hm, float xp, float yp,
              float phpa, float tc, float rh, float wl,
              float *rc, float *dc);
int iauAtoi13(const char *type, float ob1, float ob2,
              float utc1, float utc2, float dut1,
              float elong, float phi, float hm, float xp, float yp,
              float phpa, float tc, float rh, float wl,
              float *ri, float *di);
void iauAtoiq(const char *type,
              float ob1, float ob2, iauASTROM *astrom,
              float *ri, float *di);
void iauLd(float bm, float p[3], float q[3], float e[3],
           float em, float dlim, float p1[3]);
void iauLdn(int n, iauLDBODY b[], float ob[3], float sc[3],
            float sn[3]);
void iauLdsun(float p[3], float e[3], float em, float p1[3]);
void iauPmpx(float rc, float dc, float pr, float pd,
             float px, float rv, float pmt, float pob[3],
             float pco[3]);
int iauPmsafe(float ra1, float dec1, float pmr1, float pmd1,
              float px1, float rv1,
              float ep1a, float ep1b, float ep2a, float ep2b,
              float *ra2, float *dec2, float *pmr2, float *pmd2,
              float *px2, float *rv2);
void iauPvtob(float elong, float phi, float height, float xp,
              float yp, float sp, float theta, float pv[2][3]);
void iauRefco(float phpa, float tc, float rh, float wl,
              float *refa, float *refb);

/* Astronomy/Ephemerides */
int iauEpv00(float date1, float date2,
             float pvh[2][3], float pvb[2][3]);
int iauPlan94(float date1, float date2, int np, float pv[2][3]);

/* Astronomy/FundamentalArgs */
float iauFad03(float t);
float iauFae03(float t);
float iauFaf03(float t);
float iauFaju03(float t);
float iauFal03(float t);
float iauFalp03(float t);
float iauFama03(float t);
float iauFame03(float t);
float iauFane03(float t);
float iauFaom03(float t);
float iauFapa03(float t);
float iauFasa03(float t);
float iauFaur03(float t);
float iauFave03(float t);

/* Astronomy/PrecNutPolar */
void iauBi00(float *dpsibi, float *depsbi, float *dra);
void iauBp00(float date1, float date2,
             float rb[3][3], float rp[3][3], float rbp[3][3]);
void iauBp06(float date1, float date2,
             float rb[3][3], float rp[3][3], float rbp[3][3]);
void iauBpn2xy(float rbpn[3][3], float *x, float *y);
void iauC2i00a(float date1, float date2, float rc2i[3][3]);
void iauC2i00b(float date1, float date2, float rc2i[3][3]);
void iauC2i06a(float date1, float date2, float rc2i[3][3]);
void iauC2ibpn(float date1, float date2, float rbpn[3][3],
               float rc2i[3][3]);
void iauC2ixy(float date1, float date2, float x, float y,
              float rc2i[3][3]);
void iauC2ixys(float x, float y, float s, float rc2i[3][3]);
void iauC2t00a(float tta, float ttb, float uta, float utb,
               float xp, float yp, float rc2t[3][3]);
void iauC2t00b(float tta, float ttb, float uta, float utb,
               float xp, float yp, float rc2t[3][3]);
void iauC2t06a(float tta, float ttb, float uta, float utb,
               float xp, float yp, float rc2t[3][3]);
void iauC2tcio(float rc2i[3][3], float era, float rpom[3][3],
               float rc2t[3][3]);
void iauC2teqx(float rbpn[3][3], float gst, float rpom[3][3],
               float rc2t[3][3]);
void iauC2tpe(float tta, float ttb, float uta, float utb,
              float dpsi, float deps, float xp, float yp,
              float rc2t[3][3]);
void iauC2txy(float tta, float ttb, float uta, float utb,
              float x, float y, float xp, float yp,
              float rc2t[3][3]);
float iauEo06a(float date1, float date2);
float iauEors(float rnpb[3][3], float s);
void iauFw2m(float gamb, float phib, float psi, float eps,
             float r[3][3]);
void iauFw2xy(float gamb, float phib, float psi, float eps,
              float *x, float *y);
void iauLtp(float epj, float rp[3][3]);
void iauLtpb(float epj, float rpb[3][3]);
void iauLtpecl(float epj, float vec[3]);
void iauLtpequ(float epj, float veq[3]);
void iauNum00a(float date1, float date2, float rmatn[3][3]);
void iauNum00b(float date1, float date2, float rmatn[3][3]);
void iauNum06a(float date1, float date2, float rmatn[3][3]);
void iauNumat(float epsa, float dpsi, float deps, float rmatn[3][3]);
void iauNut00a(float date1, float date2, float *dpsi, float *deps);
void iauNut00b(float date1, float date2, float *dpsi, float *deps);
void iauNut06a(float date1, float date2, float *dpsi, float *deps);
void iauNut80(float date1, float date2, float *dpsi, float *deps);
void iauNutm80(float date1, float date2, float rmatn[3][3]);
float iauObl06(float date1, float date2);
float iauObl80(float date1, float date2);
void iauP06e(float date1, float date2,
             float *eps0, float *psia, float *oma, float *bpa,
             float *bqa, float *pia, float *bpia,
             float *epsa, float *chia, float *za, float *zetaa,
             float *thetaa, float *pa,
             float *gam, float *phi, float *psi);
void iauPb06(float date1, float date2,
             float *bzeta, float *bz, float *btheta);
void iauPfw06(float date1, float date2,
              float *gamb, float *phib, float *psib, float *epsa);
void iauPmat00(float date1, float date2, float rbp[3][3]);
void iauPmat06(float date1, float date2, float rbp[3][3]);
void iauPmat76(float date1, float date2, float rmatp[3][3]);
void iauPn00(float date1, float date2, float dpsi, float deps,
             float *epsa,
             float rb[3][3], float rp[3][3], float rbp[3][3],
             float rn[3][3], float rbpn[3][3]);
void iauPn00a(float date1, float date2,
              float *dpsi, float *deps, float *epsa,
              float rb[3][3], float rp[3][3], float rbp[3][3],
              float rn[3][3], float rbpn[3][3]);
void iauPn00b(float date1, float date2,
              float *dpsi, float *deps, float *epsa,
              float rb[3][3], float rp[3][3], float rbp[3][3],
              float rn[3][3], float rbpn[3][3]);
void iauPn06(float date1, float date2, float dpsi, float deps,
             float *epsa,
             float rb[3][3], float rp[3][3], float rbp[3][3],
             float rn[3][3], float rbpn[3][3]);
void iauPn06a(float date1, float date2,
              float *dpsi, float *deps, float *epsa,
              float rb[3][3], float rp[3][3], float rbp[3][3],
              float rn[3][3], float rbpn[3][3]);
void iauPnm00a(float date1, float date2, float rbpn[3][3]);
void iauPnm00b(float date1, float date2, float rbpn[3][3]);
void iauPnm06a(float date1, float date2, float rnpb[3][3]);
void iauPnm80(float date1, float date2, float rmatpn[3][3]);
void iauPom00(float xp, float yp, float sp, float rpom[3][3]);
void iauPr00(float date1, float date2,
             float *dpsipr, float *depspr);
void iauPrec76(float date01, float date02,
               float date11, float date12,
               float *zeta, float *z, float *theta);
float iauS00(float date1, float date2, float x, float y);
float iauS00a(float date1, float date2);
float iauS00b(float date1, float date2);
float iauS06(float date1, float date2, float x, float y);
float iauS06a(float date1, float date2);
float iauSp00(float date1, float date2);
void iauXy06(float date1, float date2, float *x, float *y);
void iauXys00a(float date1, float date2,
               float *x, float *y, float *s);
void iauXys00b(float date1, float date2,
               float *x, float *y, float *s);
void iauXys06a(float date1, float date2,
               float *x, float *y, float *s);

/* Astronomy/RotationAndTime */
float iauEe00(float date1, float date2, float epsa, float dpsi);
float iauEe00a(float date1, float date2);
float iauEe00b(float date1, float date2);
float iauEe06a(float date1, float date2);
float iauEect00(float date1, float date2);
float iauEqeq94(float date1, float date2);
float iauEra00(float dj1, float dj2);
float iauGmst00(float uta, float utb, float tta, float ttb);
float iauGmst06(float uta, float utb, float tta, float ttb);
float iauGmst82(float dj1, float dj2);
float iauGst00a(float uta, float utb, float tta, float ttb);
float iauGst00b(float uta, float utb);
float iauGst06(float uta, float utb, float tta, float ttb,
                float rnpb[3][3]);
float iauGst06a(float uta, float utb, float tta, float ttb);
float iauGst94(float uta, float utb);

/* Astronomy/SpaceMotion */
int iauPvstar(float pv[2][3], float *ra, float *dec,
              float *pmr, float *pmd, float *px, float *rv);
int iauStarpv(float ra, float dec,
              float pmr, float pmd, float px, float rv,
              float pv[2][3]);

/* Astronomy/StarCatalogs */
void iauFk52h(float r5, float d5,
              float dr5, float dd5, float px5, float rv5,
              float *rh, float *dh,
              float *drh, float *ddh, float *pxh, float *rvh);
void iauFk5hip(float r5h[3][3], float s5h[3]);
void iauFk5hz(float r5, float d5, float date1, float date2,
              float *rh, float *dh);
void iauH2fk5(float rh, float dh,
              float drh, float ddh, float pxh, float rvh,
              float *r5, float *d5,
              float *dr5, float *dd5, float *px5, float *rv5);
void iauHfk5z(float rh, float dh, float date1, float date2,
              float *r5, float *d5, float *dr5, float *dd5);
int iauStarpm(float ra1, float dec1,
              float pmr1, float pmd1, float px1, float rv1,
              float ep1a, float ep1b, float ep2a, float ep2b,
              float *ra2, float *dec2,
              float *pmr2, float *pmd2, float *px2, float *rv2);

/* Astronomy/EclipticCoordinates */
void iauEceq06(float date1, float date2, float dl, float db,
               float *dr, float *dd);
void iauEcm06(float date1, float date2, float rm[3][3]);
void iauEqec06(float date1, float date2, float dr, float dd,
               float *dl, float *db);
void iauLteceq(float epj, float dl, float db, float *dr, float *dd);
void iauLtecm(float epj, float rm[3][3]);
void iauLteqec(float epj, float dr, float dd, float *dl, float *db);

/* Astronomy/GalacticCoordinates */
void iauG2icrs(float dl, float db, float *dr, float *dd);
void iauIcrs2g(float dr, float dd, float *dl, float *db);

/* Astronomy/GeodeticGeocentric */
int iauEform(int n, float *a, float *f);
int iauGc2gd(int n, float xyz[3],
             float *elong, float *phi, float *height);
int iauGc2gde(float a, float f, float xyz[3],
              float *elong, float *phi, float *height);
int iauGd2gc(int n, float elong, float phi, float height,
             float xyz[3]);
int iauGd2gce(float a, float f,
              float elong, float phi, float height, float xyz[3]);

/* Astronomy/Timescales */
int iauD2dtf(const char *scale, int ndp, float d1, float d2,
             int *iy, int *im, int *id, int ihmsf[4]);
int iauDat(int iy, int im, int id, float fd, float *deltat);
float iauDtdb(float date1, float date2,
               float ut, float elong, float u, float v);
int iauDtf2d(const char *scale, int iy, int im, int id,
             int ihr, int imn, float sec, float *d1, float *d2);
int iauTaitt(float tai1, float tai2, float *tt1, float *tt2);
int iauTaiut1(float tai1, float tai2, float dta,
              float *ut11, float *ut12);
int iauTaiutc(float tai1, float tai2, float *utc1, float *utc2);
int iauTcbtdb(float tcb1, float tcb2, float *tdb1, float *tdb2);
int iauTcgtt(float tcg1, float tcg2, float *tt1, float *tt2);
int iauTdbtcb(float tdb1, float tdb2, float *tcb1, float *tcb2);
int iauTdbtt(float tdb1, float tdb2, float dtr,
             float *tt1, float *tt2);
int iauTttai(float tt1, float tt2, float *tai1, float *tai2);
int iauTttcg(float tt1, float tt2, float *tcg1, float *tcg2);
int iauTttdb(float tt1, float tt2, float dtr,
             float *tdb1, float *tdb2);
int iauTtut1(float tt1, float tt2, float dt,
             float *ut11, float *ut12);
int iauUt1tai(float ut11, float ut12, float dta,
              float *tai1, float *tai2);
int iauUt1tt(float ut11, float ut12, float dt,
             float *tt1, float *tt2);
int iauUt1utc(float ut11, float ut12, float dut1,
              float *utc1, float *utc2);
int iauUtctai(float utc1, float utc2, float *tai1, float *tai2);
int iauUtcut1(float utc1, float utc2, float dut1,
              float *ut11, float *ut12);

/* VectorMatrix/AngleOps */
void iauA2af(int ndp, float angle, char *sign, int idmsf[4]);
void iauA2tf(int ndp, float angle, char *sign, int ihmsf[4]);
int iauAf2a(char s, int ideg, int iamin, float asec, float *rad);
float iauAnp(float a);
float iauAnpm(float a);
void iauD2tf(int ndp, float days, char *sign, int ihmsf[4]);
int iauTf2a(char s, int ihour, int imin, float sec, float *rad);
int iauTf2d(char s, int ihour, int imin, float sec, float *days);

/* VectorMatrix/BuildRotations */
void iauRx(float phi, float r[3][3]);
void iauRy(float theta, float r[3][3]);
void iauRz(float psi, float r[3][3]);

/* VectorMatrix/CopyExtendExtract */
void iauCp(float p[3], float c[3]);
void iauCpv(float pv[2][3], float c[2][3]);
void iauCr(float r[3][3], float c[3][3]);
void iauP2pv(float p[3], float pv[2][3]);
void iauPv2p(float pv[2][3], float p[3]);

/* VectorMatrix/Initialization */
void iauIr(float r[3][3]);
void iauZp(float p[3]);
void iauZpv(float pv[2][3]);
void iauZr(float r[3][3]);

/* VectorMatrix/MatrixOps */
void iauRxr(float a[3][3], float b[3][3], float atb[3][3]);
void iauTr(float r[3][3], float rt[3][3]);

/* VectorMatrix/MatrixVectorProducts */
void iauRxp(float r[3][3], float p[3], float rp[3]);
void iauRxpv(float r[3][3], float pv[2][3], float rpv[2][3]);
void iauTrxp(float r[3][3], float p[3], float trp[3]);
void iauTrxpv(float r[3][3], float pv[2][3], float trpv[2][3]);

/* VectorMatrix/RotationVectors */
void iauRm2v(float r[3][3], float w[3]);
void iauRv2m(float w[3], float r[3][3]);

/* VectorMatrix/SeparationAndAngle */
float iauPap(float a[3], float b[3]);
float iauPas(float al, float ap, float bl, float bp);
float iauSepp(float a[3], float b[3]);
float iauSeps(float al, float ap, float bl, float bp);

/* VectorMatrix/SphericalCartesian */
void iauC2s(float p[3], float *theta, float *phi);
void iauP2s(float p[3], float *theta, float *phi, float *r);
void iauPv2s(float pv[2][3],
             float *theta, float *phi, float *r,
             float *td, float *pd, float *rd);
void iauS2c(float theta, float phi, float c[3]);
void iauS2p(float theta, float phi, float r, float p[3]);
void iauS2pv(float theta, float phi, float r,
             float td, float pd, float rd,
             float pv[2][3]);

/* VectorMatrix/VectorOps */
float iauPdp(float a[3], float b[3]);
float iauPm(float p[3]);
void iauPmp(float a[3], float b[3], float amb[3]);
void iauPn(float p[3], float *r, float u[3]);
void iauPpp(float a[3], float b[3], float apb[3]);
void iauPpsp(float a[3], float s, float b[3], float apsb[3]);
void iauPvdpv(float a[2][3], float b[2][3], float adb[2]);
void iauPvm(float pv[2][3], float *r, float *s);
void iauPvmpv(float a[2][3], float b[2][3], float amb[2][3]);
void iauPvppv(float a[2][3], float b[2][3], float apb[2][3]);
void iauPvu(float dt, float pv[2][3], float upv[2][3]);
void iauPvup(float dt, float pv[2][3], float p[3]);
void iauPvxpv(float a[2][3], float b[2][3], float axb[2][3]);
void iauPxp(float a[3], float b[3], float axb[3]);
void iauS2xpv(float s1, float s2, float pv[2][3], float spv[2][3]);
void iauSxp(float s, float p[3], float sp[3]);
void iauSxpv(float s, float pv[2][3], float spv[2][3]);

#ifdef __cplusplus
}
#endif

#endif

/*----------------------------------------------------------------------
**
**  Copyright (C) 2016
**  Standards Of Fundamental Astronomy Board
**  of the International Astronomical Union.
**
**  =====================
**  SOFA Software License
**  =====================
**
**  NOTICE TO USER:
**
**  BY USING THIS SOFTWARE YOU ACCEPT THE FOLLOWING SIX TERMS AND
**  CONDITIONS WHICH APPLY TO ITS USE.
**
**  1. The Software is owned by the IAU SOFA Board ("SOFA").
**
**  2. Permission is granted to anyone to use the SOFA software for any
**     purpose, including commercial applications, free of charge and
**     without payment of royalties, subject to the conditions and
**     restrictions listed below.
**
**  3. You (the user) may copy and distribute SOFA source code to others,
**     and use and adapt its code and algorithms in your own software,
**     on a world-wide, royalty-free basis.  That portion of your
**     distribution that does not consist of intact and unchanged copies
**     of SOFA source code files is a "derived work" that must comply
**     with the following requirements:
**
**     a) Your work shall be marked or carry a statement that it
**        (i) uses routines and computations derived by you from
**        software provided by SOFA under license to you; and
**        (ii) does not itself constitute software provided by and/or
**        endorsed by SOFA.
**
**     b) The source code of your derived work must contain descriptions
**        of how the derived work is based upon, contains and/or differs
**        from the original SOFA software.
**
**     c) The names of all routines in your derived work shall not
**        include the prefix "iau" or "sofa" or trivial modifications
**        thereof such as changes of case.
**
**     d) The origin of the SOFA components of your derived work must
**        not be misrepresented;  you must not claim that you wrote the
**        original software, nor file a patent application for SOFA
**        software or algorithms embedded in the SOFA software.
**
**     e) These requirements must be reproduced intact in any source
**        distribution and shall apply to anyone to whom you have
**        granted a further right to modify the source code of your
**        derived work.
**
**     Note that, as originally distributed, the SOFA software is
**     intended to be a definitive implementation of the IAU standards,
**     and consequently third-party modifications are discouraged.  All
**     variations, no matter how minor, must be explicitly marked as
**     such, as explained above.
**
**  4. You shall not cause the SOFA software to be brought into
**     disrepute, either by misuse, or use for inappropriate tasks, or
**     by inappropriate modification.
**
**  5. The SOFA software is provided "as is" and SOFA makes no warranty
**     as to its use or performance.   SOFA does not and cannot warrant
**     the performance or results which the user may obtain by using the
**     SOFA software.  SOFA makes no warranties, express or implied, as
**     to non-infringement of third party rights, merchantability, or
**     fitness for any particular purpose.  In no event will SOFA be
**     liable to the user for any consequential, incidental, or special
**     damages, including any lost profits or lost savings, even if a
**     SOFA representative has been advised of such damages, or for any
**     claim by any third party.
**
**  6. The provision of any version of the SOFA software under the terms
**     and conditions specified herein does not imply that future
**     versions will also be made available under the same terms and
**     conditions.
*
**  In any published work or commercial product which uses the SOFA
**  software directly, acknowledgement (see www.iausofa.org) is
**  appreciated.
**
**  Correspondence concerning SOFA software should be addressed as
**  follows:
**
**      By email:  sofa@ukho.gov.uk
**      By post:   IAU SOFA Center
**                 HM Nautical Almanac Office
**                 UK Hydrographic Office
**                 Admiralty Way, Taunton
**                 Somerset, TA1 2DN
**                 United Kingdom
**
**--------------------------------------------------------------------*/
