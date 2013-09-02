#ifndef POINTTOPLANEESTIMATOR3D_H_
#define POINTTOPLANEESTIMATOR3D_H_

#include "obvision/icp/IRigidEstimator.h"

namespace obvious
{

/**
 * @class PointToPlaneEstimator3D
 * @brief An estimator for registering points clouds into one common coordinate system. This estimator is based on a point-to-plane metric.
 * @author Stefan May
 */
class PointToPlaneEstimator3D : public IRigidEstimator
{
	public:
		/**
		 * Default constructor
		 */
		PointToPlaneEstimator3D();
		
		/**
		 * Destructor
		 */
		~PointToPlaneEstimator3D();

		/**
		 * Setting internal pointer to model array. The model is seen as the ground truth against which the scene has to be registered.
		 * @param model Pointer to 3 dimensional model array
		 */
		virtual void setModel(double** model, unsigned int size, double** normals);
		
		/**
		 * Setting internal pointer to scene array. (See commend for setModel)
		 * @param scene Pointer to 3 dimensional scene array
		 */
		virtual void setScene(double** scene, unsigned int size, double** normals=NULL);
		
		/**
		 * Setting assigned point pairs. You can use a pair assigner for this purpose. The deviation, i.e. the mean distance, between those pairs is also determined within this method.
		 * @param pairs Vector of pairs of indices. Each index pair references a scene and a model point.
		 */
		virtual void setPairs(std::vector<StrCartesianIndexPair>* pairs);
		
		/**
		 * Access the root mean square error that has been calculated by the setPairs method.
		 * @return RMS error
		 */
		virtual double getRMS();

		virtual unsigned int getIterations(void);
		
		/**
		 * Determine the transformation matrix that registers the scene to the model.
		 * @param T transformation matrix as return parameter
		 */
		virtual void estimateTransformation(gsl_matrix* T);
		


	private:
	
    /**
     * Pointer to model
     */
    double** _model;

    /**
     * Pointer to normals
     */
    double** _normals;

    /**
     * Pointer to scene
     */
    double** _scene;

    /**
     * Root mean square error
     */
    double _rms;
    unsigned int _iterations;

    /**
     *  Index pairs
     */
    std::vector<StrCartesianIndexPair>* _pairs;
};

}

#endif /*POINTTOPLANEESTIMATOR3D_H_*/
