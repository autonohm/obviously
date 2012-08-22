#ifndef POINTTOPOINTESTIMATOR3D_H_
#define POINTTOPOINTESTIMATOR3D_H_

#include "obvision/icp/IRigidEstimator.h"

namespace obvious
{

/**
 * @class PointToPointEstimator3D
 * @brief An estimator for registering points clouds into one common coordinate system. This estimator is based on singular value decomposition (POINTTOPOINT).
 * @author Stefan May
 */
class PointToPointEstimator3D : public IRigidEstimator
{
	public:
		/**
		 * Default constructor
		 */
		PointToPointEstimator3D();
		
		/**
		 * Destructor
		 */
		~PointToPointEstimator3D();

		/**
		 * Setting internal pointer to model array. The model is seen as the ground truth against which the scene has to be registered.
		 * @param model Pointer to 3 dimensional model array
		 */
		virtual void setModel(double** model, unsigned int size, double** normals=NULL);
		
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
		
		/**
		 * Determine the transformation matrix that registers the scene to the model.
		 * @param T transformation matrix as return parameter
		 */
		virtual void estimateTransformation(gsl_matrix* T);
		
	private:
	
		/**
		 *  Centroid of model
		 */
		double _cm[3];
		
		/**
		 * Model
		 */
		double** _model;
		
		/**
		 * Centroid of scene
		 */
		double _cs[3];
		
		/**
		 * Scene
		 */
		double** _scene;
		
		/**
		 *  Index pairs
		 */
		std::vector<StrCartesianIndexPair>* _pairs;
		
		/**
		 * Root mean square error
		 */
		double _rms;

};

}

#endif /*POINTTOPOINTESTIMATOR3D_H_*/
