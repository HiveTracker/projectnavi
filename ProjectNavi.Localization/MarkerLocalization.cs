using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace ProjectNavi.Localization
{
    public class MarkerLocalization
    {
        public Vector<double> MarkerPosition { get; set; }

        public Vector<double> SensorOffset { get; set; } // camera is not in the center of the robot

        public Vector<double> Measurement(Vector<double> mean)
        {
            // Compute position of marker relative to sensor position (not relative to to the center)
            // mean - {x, y, theta}                     // position of the robot VS its center (kalman mix of odometry + camera)
            // h - {x', y'}                             // camera measurement of the position of the marker
            // measurement = marker_pos - sensor_pos
            // sensor_pos_x = x + rs * cos(theta + theta_s)
            // sensor_pos_y = y + rs * sin(theta + theta_s)
            var result = new DenseVector(2);
            result[0] = MarkerPosition[0] - (mean[0] + SensorOffset[0] * Math.Cos(mean[2] + SensorOffset[1]));
            //     x           cst              m(x)         cst                     m(z)          cst

            result[1] = MarkerPosition[1] - (mean[1] + SensorOffset[0] * Math.Sin(mean[2] + SensorOffset[1]));
            //     y           cst              m(y)         cst                     m(z)          cst
            return result;
        }

        public Matrix<double> MeasurementJacobian(Vector<double> mean)                              // partial derivatives
        {
            var jacobian = new[,] {{-1, 0, SensorOffset[0] * -Math.Sin(mean[2] + SensorOffset[1])}, // d(result[0])/dx, d(result[0])/dy, d(result[0])/dz
                                   {0, -1, SensorOffset[0] * Math.Cos(mean[2] + SensorOffset[1])}}; // d(result[1])/dx, d(result[1])/dy, d(result[1])/dz
            return DenseMatrix.OfArray(jacobian);
        }

        public void MarkerUpdate(KalmanFilter kalman, Vector<double> measurement)
        {
            var noise = DenseMatrix.OfArray(new [,]{{0.01, 0},      // 0.01m = 10cm noise (1st guess that worked)
                                                    {0, 0.01},});
            kalman.Update(measurement, Measurement, MeasurementJacobian, noise);
        }
    }
}
