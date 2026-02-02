#define	XP2_FLOAT	float
#define	XP2_CONTROL_CURVE_LINEAR	1.0f	// sigmoid curve maximum before switching to linear.
#define XP2_CONTROL_CURVE_MIN       0.0001f // zero is not allowed.
#define	XP2_CONTROL_CURVE_POW		4.0f	// used for pow() function
#define	XP2_CONTROL_CURVE_FAST_POW	4		// must be whole number, 1,2,3,4,5 etc


// *******************************************************
//	calcualtes a value for K based on an input
//	value of -1 to +1
// *******************************************************

XP2_FLOAT sigmoidCalcK( XP2_FLOAT *pK )
{
	XP2_FLOAT	lK;

	// check range, return linear if error.
	if( *pK < -XP2_CONTROL_CURVE_LINEAR || *pK > XP2_CONTROL_CURVE_LINEAR )
		{
		*pK = XP2_CONTROL_CURVE_LINEAR;
		return 0.0f;
		}

	//	calculate
	lK = fabs(*pK) * 2.0f;
#ifdef XP2_CONTROL_CURVE_FAST_POW
#if XP2_CONTROL_CURVE_FAST_POW == 4
	lK = lK*lK*lK*lK;
#else
	error function needs definition!
#endif
#else
	lK = pow(lK,XP2_CONTROL_CURVE_POW);
#endif
	//	limit minimum size of K
	if( lK < XP2_CONTROL_CURVE_MIN  ) lK = XP2_CONTROL_CURVE_MIN ;

	//	check signbit and invert/offset K if needed
	if( signbit(*pK) ) lK = -lK - 1.0f;

	return lK;
}

// *******************************************************
// K Must be 0 to Infinity or -1 to Infinity
// when K = INFINITY or -INFINITY, the line is straight
// input = -1 to 1, output = -1 to 1

// TODO may need improving, take a look at these equations:
// http://math.stackexchange.com/questions/459872/adjustable-sigmoid-curve-s-curve-from-0-0-to-1-1

XP2_FLOAT sigmoidCurve( XP2_FLOAT X, XP2_FLOAT K )
{

// get scaled value for K
XP2_FLOAT lK = sigmoidCalcK(&K);

// once K approaches 25, curve is near linear, so return linear value
if( fabs(K) >= XP2_CONTROL_CURVE_LINEAR ) return X;

XP2_FLOAT T = fabs(X);
XP2_FLOAT Y;

// calculate Y
Y = (lK*T) / (1.0f+lK-T);

// check for negative Y
if( X < 0.0f ) return -Y;
else return Y;

}

XP2_FLOAT bipolarSigmoidCurve( XP2_FLOAT X, XP2_FLOAT K )
{
// convert to +/-1.0 range
X -= 0.5f;
X *= 2.0f;

XP2_FLOAT T = sigmoidCurve( X, K );

// convert to 0 to 1.0 range

T *= 0.5f;
T += 0.5f;
return T;
}
