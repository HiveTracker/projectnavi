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
/*
            var result = new DenseVector(2);
            result[0] = MarkerPosition[0] - (mean[0] + SensorOffset[0] * Math.Cos(mean[2] + SensorOffset[1]));
            //     x           cst              m(x)         cst                     m(z)          cst

            result[1] = MarkerPosition[1] - (mean[1] + SensorOffset[0] * Math.Sin(mean[2] + SensorOffset[1]));
            //     y           cst              m(y)         cst                     m(z)          cst

            return result;
*/
            ///////////////////////////////////////////////////////////////////////////////////////

            // For explanations, see schematics & calculations:
            // TODO: use github readme instead of gDoc!
            // https://docs.google.com/document/d/19Me8gSwOVkv-zJMuCdPM1gr30l-nCvr4GXn-jcXUsUs/

            // TODO: put these variables/constants in a global file
            const int bases_num = 2;
            const int dimensions_num = 2;                              // TODO: think in 3d later?
            const int photodiodes_num = 4;
            const float sensorOffset[photodiodes_num][dimensions_num]; // (meters) this hard coded
//          float sweepMeasure[bases_num][photodiodes_num] = mean;     // (radians) given as argument
            float baseRotation[bases_num][photodiodes_num];            // (radians) what we want
            float basePosition[bases_num][dimensions_num];             // (meters)  what we want too!
            // TODO: investigate: how do we initialize basePosition?
            // TODO: investigate: do we compute basePosition with a kind of gradient descent!?

            for (int b = 0; b < bases_num; b++)
            {
                for (int p = 0; p < photodiodes_num; p++)
                {
                    // r = atan( (x-xi) / (y-y1) ) - φi
                    baseRotation[b][p] = atan2( (basePosition[b][0] - sensorOffset[p][0]) , // x - xi
                                                (basePosition[b][1] - sensorOffset[p][1]) ) // y - yi
                                       - sweepMeasure[b][p];                                // - φi
                }
            }

            return baseRotation;

            ///////////////////////////////////////////////////////////////////////////////////////
        }

        public Matrix<double> MeasurementJacobian(Vector<double> mean)                              // partial derivatives
        {
            // TODO: double jacobian (1 for each base?)
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
