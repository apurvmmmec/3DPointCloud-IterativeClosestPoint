
#include "acq/normalEstimation.h"
#include "acq/decoratedCloud.h"
#include "acq/cloudManager.h"
#include <igl/read_triangle_mesh.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/combine.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Geometry>
#include <thread>
#include <acq/myicp.h>
#include <acq/icpNormal.h>
#include <acq/icpMultimesh.h>
#include <acq/icpSubsampled.h>
#include <thread>
#include <submain.h>
#include <igl/per_corner_normals.h>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace acq;

int numMesh=2;

vector<MatrixXd> vecV(numMesh);
vector<MatrixXi> vecF(numMesh);

igl::viewer::Viewer viewer;
MatrixXd V;
MatrixXi F;
MatrixXd C;
float maxNeighbourDist = 0.15;
int kNeighbours = 5;
bool bColorByNormals=false;








namespace acq {
    
    /** \brief                      Re-estimate normals of cloud \p V fitting planes
     *                              to the \p kNeighbours nearest neighbours of each point.
     * \param[in ] kNeighbours      How many neighbours to use (Typiclaly: 5..15)
     * \param[in ] vertices         Input pointcloud. Nx3, where N is the number of points.
     * \param[in ] maxNeighbourDist Maximum distance between vertex and neighbour.
     * \param[out] viewer           The viewer to show the normals at.
     * \return                      The estimated normals, Nx3.
     */
    NormalsT
    recalcNormals(
                  int                 const  kNeighbours,
                  CloudT              const& vertices,
                  float               const  maxNeighbourDist
                  ) {
        NeighboursT const neighbours =
        calculateCloudNeighbours(
                                 /* [in]        cloud: */ vertices,
                                 /* [in] k-neighbours: */ kNeighbours,
                                 /* [in]      maxDist: */ maxNeighbourDist
                                 );
        
        // Estimate normals for points in cloud vertices
        NormalsT normals =
        calculateCloudNormals(
                              /* [in]               Cloud: */ vertices,
                              /* [in] Lists of neighbours: */ neighbours
                              );
        
        return normals;
    } //...recalcNormals()
    
    void setViewerNormals(
                          igl::viewer::Viewer      & viewer,
                          CloudT              const& vertices,
                          NormalsT            const& normals
                          ) {
        // [Optional] Set viewer face normals for shading
        //viewer.data.set_normals(normals);
        
        // Clear visualized lines (see Viewer.clear())
        viewer.data.lines = Eigen::MatrixXd(0, 9);
        
        // Add normals to viewer
        viewer.data.add_edges(
                              /* [in] Edge starting points: */ vertices,
                              /* [in]       Edge endpoints: */ vertices + normals * 0.01, // scale normals to 1% length
                              /* [in]               Colors: */ Eigen::Vector3d::Zero()
                              );
    }
    
} //...ns acq


// Callback function called from the ICP functions when convergence is done in order to color the
// unmatched vertices of mesh 2 which did not find match in P

void colorUnMatched1(vector<int> unMatchIdx){
    //cout<<"IN Unmatched";
    int nR1 = vecV[0].rows();
    
    //Just color the vertices of mesh2 with red for all vertices of unMatchIdx vector
    for (int i=0;i<unMatchIdx.size();i++){
            C.row(unMatchIdx.at(i)+nR1) = Eigen::RowVector3d(1,0,0);
    }
    viewer.data.set_colors(C);
}

//Call back function called from ICP function to upadate the transformed meshes at end of each ICP iterartion
void updateView1(vector<MatrixXd> vecV){
    //    cout<<"Inside Update View";
    igl::combine(vecV,vecF,V,F);
    
    //If we wish to color the meshes according to normal color, then call this function
    if(bColorByNormals){
        int nR1 = vecV[0].rows();
        int nR2 = vecV[1].rows();
        int nR  = V.rows();
        MatrixXd normP(vecV[0].rows(),3);
        normP = recalcNormals(kNeighbours,vecV[0],maxNeighbourDist);
        
        MatrixXd normQ(vecV[1].rows(),3);
        normQ = recalcNormals(kNeighbours,vecV[1],maxNeighbourDist);
        for (int i=0;i<nR;i++){
            if(i<nR1){
                C(i,0) = abs(normP(i,0));
                C(i,1) = abs(normP(i,1));
                C(i,2) = abs(normP(i,2));
                
            }else{
                C(i,0) = abs(normQ(i-nR1,0));
                C(i,1) = abs(normQ(i-nR1,1));
                C(i,2) = abs(normQ(i-nR1,2));
            }
        }
    }
    viewer.data.set_colors(C);
    viewer.data.set_mesh(V,F);
}

int main(int argc, char * argv[])
{

    CloudManager cloudManager;
    
    // We set the num mesh to 2 for all cases except when we do multiple mesh alignment
    if(numMesh==2){
    
        string meshPath[]={"./bun000.off","./bun045.off"};
        for(int i = 0;i<2;i++)
        {
            igl::read_triangle_mesh(meshPath[i],vecV[i],vecF[i]);
        }
        
        int nR1 = vecV[0].rows();
        int nR2 = vecV[1].rows();
        igl::combine(vecV,vecF,V,F);
        int nR  = V.rows();
        C.resize(nR,3);
        for (int i=0;i<nR;i++){
            if(i<nR1){
                C.row(i) = Eigen::RowVector3d(0,1,0);
            }else{
                C.row(i) = Eigen::RowVector3d(0,0,1);
            }
        }
    }else if(numMesh==5){
        
        string meshPath[]={"./bun000.off"
            ,"./bun000.off"
            ,"./bun000.off",
        "./bun000.off",
        "./bun000.off"};
        
        for(int i = 0;i<numMesh;i++)
        {
            igl::read_triangle_mesh(meshPath[i],vecV[i],vecF[i]);
        }
        
        int nR1 = vecV[0].rows();
        int nR2 = vecV[1].rows();
        int nR3 = vecV[3].rows();
        int nR4 = vecV[4].rows();
        
        igl::combine(vecV,vecF,V,F);
        int nR  = V.rows();
        
        C.resize(nR,3);
        for (int i=0;i<nR;i++){
            if(i<nR1){
                C.row(i) = Eigen::RowVector3d(0,1,0);
            }if(i>=nR1 && i<(nR1+nR2)){
                C.row(i) = Eigen::RowVector3d(0,0,1);
            }if(i>=(nR1+nR2) && i<(nR1+nR2+nR3)){
                C.row(i) = Eigen::RowVector3d(0.5,0.5,1);
            }if(i>=(nR1+nR2+nR3) && i<(nR1+nR2+nR3+nR4)){
                C.row(i) = Eigen::RowVector3d(0.3,0.9,0.2);
            }
            if(i>=(nR1+nR2+nR3+nR4)){
                C.row(i) = Eigen::RowVector3d(1,0,0);
            }
        }
    }
    
    
    viewer.data.set_mesh(V,F);
    viewer.data.set_colors(C);
    
    //Here we combine all the vertices and faces in vector vecV and vecF to a create combined
    // matrices V and F which we show in viewer
    igl::combine(vecV,vecF,V,F);
    viewer.data.set_mesh(V,F);
    viewer.core.is_animating=true;
    viewer.core.show_lines = 0;
    
    
    // Store read vertices and faces
    cloudManager.addCloud(DecoratedCloud(V, F));
    
    
    //Please uncomment the appriate line to run required function
    
    // Question 1-3
    //thread t1(icpPointToPoint,vecV,&updateView1,&colorUnMatched1);
    
    
    //Question 4
    //thread t1(icpSubsampled,vecV,0.001, &updateView1);
    
    //Question 5
    //thread t1(icpMultimesh,vecV,&updateView1);

    //Question 6
            MatrixXd normQ(vecV[1].rows(),3);
            normQ = recalcNormals(kNeighbours,vecV[1],maxNeighbourDist);
            bColorByNormals=true;
            updateView1(vecV);
        thread t1(icpNormal,vecV,normQ,&updateView1);
    

    //createGUI(cloudManager,kNeighbours, maxNeighbourDist);
    

    viewer.launch();
    
}

