
#include <AP_Math/AP_Math.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AP_Chebyshev_II_SP_2000_PASS_70_STOP_140_DB_30
#define AP_Chebyshev_II_SP_2000_PASS_60_STOP_120_DB_30
#define AP_Chebyshev_II_SP_2000_PASS_50_STOP_100_DB_30
#define AP_Chebyshev_II_SP_2000_PASS_40_STOP_80_DB_30
#define AP_Chebyshev_II_SP_2000_PASS_30_STOP_60_DB_30
// #define AP_Chebyshev_II_SP_2000_PASS_25_STOP_50_DB_30
// #define AP_Chebyshev_II_SP_2000_PASS_20_STOP_40_DB_30
// #define AP_Chebyshev_II_SP_2000_PASS_10_STOP_30_DB_30

// #define AP_Butterworth_SP_2000_PASS_25_STOP_50_DB_30
// #define AP_Butterworth_SP_2000_PASS_10_STOP_30_DB_60
// #define AP_Butterworth_SP_2000_PASS_5_STOP_20_DB_60
// #define AP_Butterworth_SP_2000_PASS_5_STOP_10_DB_60

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef AP_Chebyshev_II_SP_2000_PASS_70_STOP_140_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_70_STOP_140_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T denAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        denAccum = (xin * 0.0434932634F + Hlp_FILT_STATES[0] * 1.76398361F) - Hlp_FILT_STATES[1] * 0.839866042F;
        rtb_Hlp = (denAccum * 0.691343248F - Hlp_FILT_STATES[0] * 1.22976267F) + Hlp_FILT_STATES[1] * 0.691343248F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[2] * 1.45252705F) - Hlp_FILT_STATES[3] * 0.542354882F;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = (denAccum * 1.00716901F - Hlp_FILT_STATES[2] * 0.989500165F) + Hlp_FILT_STATES[3] * 1.00716901F;

        /* S-Function (sdspbiquad): '<Root>/Hlp' */
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = denAccum;

        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_60_STOP_120_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_60_STOP_120_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T denAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        denAccum = (xin * 0.0323699936F + Hlp_FILT_STATES[0] * 1.80477607F) - Hlp_FILT_STATES[1] * 0.860991776F;
        rtb_Hlp = (denAccum * 0.707323492F - Hlp_FILT_STATES[0] * 1.29895806F) + Hlp_FILT_STATES[1] * 0.707323492F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[2] * 1.52519429F) - Hlp_FILT_STATES[3] * 0.593317628F;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = (denAccum * 1.2845341F - Hlp_FILT_STATES[2] * 1.54643643F) + Hlp_FILT_STATES[3] * 1.2845341F;

        /* S-Function (sdspbiquad): '<Root>/Hlp' */
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = denAccum;

        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_40_STOP_80_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_40_STOP_80_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T numAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        rtb_Hlp = (xin * 0.0147584584F + Hlp_FILT_STATES[0] * 1.87950945F) - Hlp_FILT_STATES[1] * 0.904962F;
        numAccum = (rtb_Hlp * 0.746903956F - Hlp_FILT_STATES[0] * 1.43897283F) + Hlp_FILT_STATES[1] * 0.746903956F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[2] * 1.67540455F) - Hlp_FILT_STATES[3] * 0.707817F;
        numAccum = (rtb_Hlp * 1.24460757F - Hlp_FILT_STATES[2] * 2.0F) + Hlp_FILT_STATES[3] * 1.24460757F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = rtb_Hlp;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = numAccum * 2.08374548F;

        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_50_STOP_100_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_50_STOP_100_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T numAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        rtb_Hlp = (xin * 0.0227687694F + Hlp_FILT_STATES[0] * 1.84329855F) - Hlp_FILT_STATES[1] * 0.882688224F;
        numAccum = (rtb_Hlp * 0.725771844F - Hlp_FILT_STATES[0] * 1.3686589F) + Hlp_FILT_STATES[1] * 0.725771844F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[2] * 1.59943473F) - Hlp_FILT_STATES[3] * 0.648343205F;
        numAccum = (rtb_Hlp * 1.41340518F - Hlp_FILT_STATES[2] * 2.0F) + Hlp_FILT_STATES[3] * 1.41340518F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = rtb_Hlp;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = numAccum * 1.23466039F;
        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_30_STOP_60_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_30_STOP_60_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T numAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        rtb_Hlp = (xin * 0.0084072664136534657f + Hlp_FILT_STATES[0] * 1.9133564380936847f) - Hlp_FILT_STATES[1] * 0.9278208369479326f;
        numAccum = (rtb_Hlp * 0.77098298853080771f - Hlp_FILT_STATES[0] * 1.5100159920311977f) + Hlp_FILT_STATES[1] * 0.7709829885308076f;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[2] * 1.7532681726062895f) - Hlp_FILT_STATES[3] * 0.7721788425848245f;
        numAccum = (rtb_Hlp * 1.1299605771807173f - Hlp_FILT_STATES[2] * 2.0f) + Hlp_FILT_STATES[3] * 1.1299605771807171f;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = rtb_Hlp;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = numAccum * 3.9177814767596457f;

        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_25_STOP_50_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_25_STOP_50_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T numAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        rtb_Hlp = (xin * 0.0058752355182779591f + Hlp_FILT_STATES[0] * 1.929373685504771f) - Hlp_FILT_STATES[1] * 0.93947240725544578f;
        numAccum = (rtb_Hlp * 0.7842265758607817f - Hlp_FILT_STATES[0] * 1.5458536427243097f) + Hlp_FILT_STATES[1] * 0.78422657586078159f;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[2] * 1.7929639941952533f) - Hlp_FILT_STATES[3] * 0.80634722719126184f;
        numAccum = (rtb_Hlp * 1.0883257268080553f - Hlp_FILT_STATES[2] * 2.0f) + Hlp_FILT_STATES[3] * 1.0883257268080555f;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = rtb_Hlp;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = numAccum * 5.7621683857646779f;

        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_20_STOP_40_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_20_STOP_40_DB_30
{
public:
    T Hlp_FILT_STATES[4]; /* '<Root>/Hlp' */
    T numAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        rtb_Hlp = (xin * 0.0037838628F + Hlp_FILT_STATES[0] * 1.94477463F) - Hlp_FILT_STATES[1] * 0.95127362F;
        numAccum = (rtb_Hlp * 0.798321545F - Hlp_FILT_STATES[0] * 1.58190298F) + Hlp_FILT_STATES[1] * 0.798321545F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[2] * 1.8331989F) - Hlp_FILT_STATES[3] * 0.841931701F;
        numAccum = (rtb_Hlp * 1.05555904F - Hlp_FILT_STATES[2] * 2.0f) + Hlp_FILT_STATES[3] * 1.05555904F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = rtb_Hlp;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = numAccum * 9.15742302F;

        return Out1;
    }
};
#endif

#ifdef AP_Chebyshev_II_SP_2000_PASS_10_STOP_30_DB_30
template <class T>
class Chebyshev_II_SP_2000_PASS_10_STOP_30_DB_30
{
public:
    T Hlp_FILT_STATES[6]; /* '<Root>/Hlp' */
    T numAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &xin)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        rtb_Hlp = (xin * 0.0424023643F + Hlp_FILT_STATES[0] * 0.957597613F);
        numAccum = (rtb_Hlp * 0.000547178322F + Hlp_FILT_STATES[0] * 0.000547178322F);
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[2] * 1.97621727F) - Hlp_FILT_STATES[3] * 0.977775156F;
        numAccum = (rtb_Hlp * 0.240602896F - Hlp_FILT_STATES[2] * 0.478845298F) + Hlp_FILT_STATES[3] * 0.240602896F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = rtb_Hlp;
        rtb_Hlp = (numAccum + Hlp_FILT_STATES[4] * 1.93506753F) - Hlp_FILT_STATES[5] * 0.936760724F;
        numAccum = (rtb_Hlp * 1.01295757F - Hlp_FILT_STATES[4] * 2.0F) + Hlp_FILT_STATES[5] * 1.01295757F;
        Hlp_FILT_STATES[5] = Hlp_FILT_STATES[4];
        Hlp_FILT_STATES[4] = rtb_Hlp;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        return Out1 = numAccum * 39.400383F;
    }
};
#endif

#ifdef AP_Butterworth_SP_2000_PASS_25_STOP_50_DB_30
template <class T>
class Butterworth_SP_2000_PASS_25_STOP_50_DB_30
{
public:
    T Hlp_FILT_STATES[6]; /* '<Root>/Hlp' */
    T denAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &In1)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        denAccum = (In1 * 0.0072058565F + Hlp_FILT_STATES[0] * 1.83553743F) - Hlp_FILT_STATES[1] * 0.842743337F;
        rtb_Hlp = (denAccum * 0.00165310863F + Hlp_FILT_STATES[0] * 0.00330621726F) + Hlp_FILT_STATES[1] * 0.00165310863F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[2] * 1.94764376F) - Hlp_FILT_STATES[3] * 0.955289662F;
        rtb_Hlp = (denAccum * 0.00212706253F + Hlp_FILT_STATES[2] * 0.00425412506F) + Hlp_FILT_STATES[3] * 0.00212706253F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[4] * 1.8750416F) - Hlp_FILT_STATES[5] * 0.882402539F;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = (denAccum * 0.250094146F + Hlp_FILT_STATES[4] * 0.500188291F) + Hlp_FILT_STATES[5] * 0.250094146F;

        /* S-Function (sdspbiquad): '<Root>/Hlp' */
        Hlp_FILT_STATES[5] = Hlp_FILT_STATES[4];
        Hlp_FILT_STATES[4] = denAccum;

        return Out1;
    }
};
#endif

#ifdef AP_Butterworth_SP_2000_PASS_10_STOP_30_DB_60
template <class T>
class Butterworth_SP_2000_PASS_10_STOP_30_DB_60
{
public:
    T Hlp_FILT_STATES[8]; /* '<Root>/Hlp' */
    T denAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &In1)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        denAccum = (In1 * 0.00119775871F + Hlp_FILT_STATES[0] * 1.93741369F) - Hlp_FILT_STATES[1] * 0.938611388F;
        rtb_Hlp = (denAccum * 0.000221711613F + Hlp_FILT_STATES[0] * 0.000443423225F) + Hlp_FILT_STATES[1] * 0.000221711613F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[2] * 1.98325348F) - Hlp_FILT_STATES[3] * 0.984479547F;
        rtb_Hlp = (denAccum * 0.0114420829F + Hlp_FILT_STATES[2] * 0.0228841659F) + Hlp_FILT_STATES[3] * 0.0114420829F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[4]) * 0.965449572F;
        rtb_Hlp = (denAccum * 0.000630964583F + Hlp_FILT_STATES[4] * 0.000630964583F);
        Hlp_FILT_STATES[5] = Hlp_FILT_STATES[4];
        Hlp_FILT_STATES[4] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[6] * 1.95590317F) - Hlp_FILT_STATES[7] * 0.957112372F;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = (denAccum * 0.250013977F + Hlp_FILT_STATES[6] * 0.500027955F) + Hlp_FILT_STATES[7] * 0.250013977F;

        /* S-Function (sdspbiquad): '<Root>/Hlp' */
        Hlp_FILT_STATES[7] = Hlp_FILT_STATES[6];
        Hlp_FILT_STATES[6] = denAccum;

        return Out1;
    }
};
#endif

#ifdef AP_Butterworth_SP_2000_PASS_5_STOP_20_DB_60
template <class T>
class Butterworth_SP_2000_PASS_5_STOP_20_DB_60
{
public:
    T Hlp_FILT_STATES[6]; /* '<Root>/Hlp' */
    T denAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &In1)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        denAccum = (In1 * 0.000387565204F + Hlp_FILT_STATES[0] * 1.96194243F) - Hlp_FILT_STATES[1] * 0.96233F;
        rtb_Hlp = (denAccum * 8.50990255E-5F + Hlp_FILT_STATES[0] * 0.000170198051F) + Hlp_FILT_STATES[1] * 8.50990255E-5F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[2] * 1.98937225F) - Hlp_FILT_STATES[3] * 0.989765227F;
        rtb_Hlp = (denAccum * 0.000112426234F + Hlp_FILT_STATES[2] * 0.000224852469F) + Hlp_FILT_STATES[3] * 0.000112426234F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[4] * 1.97189426F) - Hlp_FILT_STATES[5] * 0.972283781F;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = (denAccum * 0.250002593F + Hlp_FILT_STATES[4] * 0.500005186F) + Hlp_FILT_STATES[5] * 0.250002593F;

        /* S-Function (sdspbiquad): '<Root>/Hlp' */
        Hlp_FILT_STATES[5] = Hlp_FILT_STATES[4];
        Hlp_FILT_STATES[4] = denAccum;

        return Out1;
    }
};
#endif

#ifdef AP_Butterworth_SP_2000_PASS_5_STOP_10_DB_60
template <class T>
class Butterworth_SP_2000_PASS_5_STOP_10_DB_60
{
public:
    T Hlp_FILT_STATES[12]; /* '<Root>/Hlp' */
    T denAccum;
    T rtb_Hlp;
    T Out1;

    T apply(T &In1)
    {
        /* S-Function (sdspbiquad): '<Root>/Hlp' incorporates:
         *  Inport: '<Root>/In1'
         */
        denAccum = (In1 * 0.000275240454F + Hlp_FILT_STATES[0] * 1.97800171F) - Hlp_FILT_STATES[1] * 0.978279769F;
        rtb_Hlp = (denAccum * 6.98758304E-5F + Hlp_FILT_STATES[0] * 0.000139751661F) + Hlp_FILT_STATES[1] * 6.98758304E-5F;
        Hlp_FILT_STATES[1] = Hlp_FILT_STATES[0];
        Hlp_FILT_STATES[0] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[2] * 1.96805906F) - Hlp_FILT_STATES[3] * 0.968335807F;
        rtb_Hlp = (denAccum * 4.57058668E-5F + Hlp_FILT_STATES[2] * 9.14117336E-5F) + Hlp_FILT_STATES[3] * 4.57058668E-5F;
        Hlp_FILT_STATES[3] = Hlp_FILT_STATES[2];
        Hlp_FILT_STATES[2] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[4] * 1.99495888F) - Hlp_FILT_STATES[5] * 0.995239258F;
        rtb_Hlp = (denAccum * 0.000106307205F + Hlp_FILT_STATES[4] * 0.00021261441F) + Hlp_FILT_STATES[5] * 0.000106307205F;
        Hlp_FILT_STATES[5] = Hlp_FILT_STATES[4];
        Hlp_FILT_STATES[4] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[6] * 1.9719063F) - Hlp_FILT_STATES[7] * 0.972183526F;
        rtb_Hlp = (denAccum * 0.00415692898F + Hlp_FILT_STATES[6] * 0.00831385795F) + Hlp_FILT_STATES[7] * 0.00415692898F;
        Hlp_FILT_STATES[7] = Hlp_FILT_STATES[6];
        Hlp_FILT_STATES[6] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[8] * 0.983372271F);
        rtb_Hlp = (denAccum * 0.000139583106F + Hlp_FILT_STATES[8] * 0.000139583106F);
        Hlp_FILT_STATES[9] = Hlp_FILT_STATES[8];
        Hlp_FILT_STATES[8] = denAccum;
        denAccum = (rtb_Hlp + Hlp_FILT_STATES[10] * 1.98588753F) - Hlp_FILT_STATES[11] * 0.986166716F;

        /* Outport: '<Root>/Out1' incorporates:
         *  S-Function (sdspbiquad): '<Root>/Hlp'
         */
        Out1 = (denAccum * 0.250005186F + Hlp_FILT_STATES[10] * 0.500010371F) + Hlp_FILT_STATES[11] * 0.250005186F;

        /* S-Function (sdspbiquad): '<Root>/Hlp' */
        Hlp_FILT_STATES[11] = Hlp_FILT_STATES[10];
        Hlp_FILT_STATES[10] = denAccum;

        return Out1;
    }
};
#endif
