//
//  nearestNeighbour.cpp
//  IGLFramework
//
//  Created by Apurv Nigam on 15/02/2017.
//
//

#include <stdio.h>
#include <iostream>
#include <igl/point_mesh_squared_distance.h>
#include <acq/nearestNeighbour.h>
#include <ANN/ANN.h>					// ANN declarations

using namespace std;
using namespace Eigen;


//Function to calculate the nearest neighbours in mesh Q for each point in mesh P
map<int,int> findClosestPoints(MatrixXd vp, MatrixXd vq){
    
    ANNpointArray		dataPts;				// data points
    ANNpoint			queryPt;				// query point
    ANNidxArray			nnIdx;					// near neighbor indices
    ANNdistArray		dists;					// near neighbor distances
    ANNkd_tree*			kdTree;					// search structure
    
    MatrixXd p = vp.transpose();
    MatrixXd q = vq.transpose();
    
    int				k				= 1;			// number of nearest neighbors
    int				dim				= 3;			// dimension
    double			eps				= 0;			// error bound
    
    int numQ = q.cols();
    int numP = p.cols();
    
    queryPt = annAllocPt(dim);					// allocate query point
    dataPts = annAllocPts(numQ, dim);			// allocate data points
    nnIdx = new ANNidx[k];						// allocate near neigh indices
    dists = new ANNdist[k];						// allocate near neighbor dists
    map<int,int> nearNb;
    
    
    for (int r = 0 ; r< numQ;r++){
        dataPts[r][0] = q.col(r)(0);
        dataPts[r][1] = q.col(r)(1);
        dataPts[r][2] = q.col(r)(2);
    }
    
    
    kdTree = new ANNkd_tree(dataPts,numQ,dim);
    
    for (int i=0;i<numP;i++){
        queryPt[0] = p.col(i)(0);queryPt[1] = p.col(i)(1);queryPt[2] = p.col(i)(2);
        
        kdTree->annkSearch(
                           queryPt,	// query point
                           k,		// number of near neighbors
                           nnIdx,	// nearest neighbors (returned)
                           dists,	// distance (returned)
                           eps);	// error bound
        
        
        nearNb[i] = nnIdx[0];
        
    }
    
    //Free up the memory
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    delete dataPts[0];
    delete[] dataPts;
    delete queryPt;
    
    //Close ANN
    annClose();
    
    
    //Return the map containing the correspondances
    return nearNb;
}

