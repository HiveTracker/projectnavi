using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

namespace ProjectNavi.Localization
{
    public class KalmanFilter
    {
        public Vector<double> State { get; set; }

        public Matrix<double> Uncertainty { get; set; }

        public void Predict(Vector<double> control, StateTransitionFunc stateTransition, StateTransitionJacobianFunc stateTransitionJacobian, Matrix<double> noise)
        {
            var jacobian = stateTransitionJacobian(control, State);
            State = stateTransition(control, State);
            Uncertainty = jacobian * Uncertainty * jacobian.Transpose() + noise;
        }

        public void Update(Vector<double> measurement, MeasurementFunc measurementFunction, MeasurementJacobianFunc measurementFunctionJacobian, Matrix<double> noise)
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
