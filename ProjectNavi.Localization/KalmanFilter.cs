using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

namespace ProjectNavi.Localization
{
    public class KalmanFilter // EXTENDED
    {
        public Vector<double> State { get; set; }

        public Matrix<double> Uncertainty { get; set; } // same dimension as above, but square
        // TODO: dig how do we see the gaussian here?

        // estimate with inacurate measures for prediction
        public void Predict(Vector<double> control,                              // measured values of sensor (ex: accelerometer)
                            StateTransitionFunc stateTransition,                 // estimation function (ex: extrapolation)
                            StateTransitionJacobianFunc stateTransitionJacobian, // uncertainty update (for compensation TODO: check gonçalo's video when it's described)
                            Matrix<double> noise)                                // sensor characteristics (variances in diagonal)
        {
            var jacobian = stateTransitionJacobian(control, State);              // matrix of partial derivatives helping to anticipate possible future states (confidence...)
            State = stateTransition(control, State);                             // could be non-linear => extended TODO: look for polynomial extrapolation
            Uncertainty = jacobian * Uncertainty * jacobian.Transpose() + noise; // covariance matrix (amplify similarities - not symetrical)
        }

        // AKA corrector function - update with measures to decrease error:
        public void Update(Vector<double> measurement,                           // φi: usually more accurate but less frequent
                           MeasurementFunc measurementFunction,                  // Fi(Θ) = atan((x-xi)/(y-yi))-r
                           MeasurementJacobianFunc measurementFunctionJacobian,  // ∇ Fi (NOT Ji)
                           Matrix<double> noise)                                 // TODO: measure photosensor noise (co)variance (can we integrate the position dependant distortion)
        {
            var jacobian = measurementFunctionJacobian(State);
            var kalmanGain = Uncertainty * jacobian.Transpose() * (jacobian * Uncertainty * jacobian.Transpose() + noise).Inverse();
            State = State + kalmanGain * (measurement - measurementFunction(State));
            Uncertainty = (DenseMatrix.CreateIdentity(State.Count) - kalmanGain * jacobian) * Uncertainty;
        }
    }

    public delegate Vector<double> StateTransitionFunc(Vector<double> control, Vector<double> mean);

    public delegate Matrix<double> StateTransitionJacobianFunc(Vector<double> control, Vector<double> mean);

    public delegate Vector<double> MeasurementFunc(Vector<double> mean);

    public delegate Matrix<double> MeasurementJacobianFunc(Vector<double> mean);
}

