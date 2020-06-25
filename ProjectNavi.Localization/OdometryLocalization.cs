using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace ProjectNavi.Localization
{
    public static class OdometryLocalization
    {
        public static Vector<double> StateTransitionFunc(Vector<double> control, Vector<double> mean)
        {
            var result = mean.Clone();

            result[0] += control[0] * Math.Cos(mean[2]);
            result[1] += control[0] * Math.Sin(mean[2]);
            result[2] += control[1];
            //result[2] = MathHelper.WrapAngle((float)result[2]);
            return result;
        }

        public static Matrix<double> StateTransitionJacobianFunc(Vector<double> control, Vector<double> mean)
        {
            var jacob = new[,] { {1, 0, -control[0] * Math.Sin(mean[2])},
                                 {0, 1, control[0] * Math.Cos(mean[2])},
                                 {0, 0, 1}};
            Matrix<double> result = DenseMatrix.OfArray(jacob);
            return result;
        }

        public static void OdometerPredict(KalmanFilter kalman, double dx, double dtheta)
        {
            var tmp = new double[] { dx, dtheta };
            Vector<double> control = new DenseVector(tmp);
            var noise = new double[,] { {0.1, 0, 0},    // x: 0.1m = 10 cm
                                        {0, 0.1, 0},    // y: 0.1m = 10 cm
                                        {0, 0, 0.2}};   // theta: 0.2 radian
            kalman.Predict(control, StateTransitionFunc, StateTransitionJacobianFunc, DenseMatrix.OfArray(noise));
        }
    }
}
