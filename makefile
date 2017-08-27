#compiler
OMPL_DIR = /usr/local
INC_CLASSES = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/proj_classes/
INC_PLANNERS = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/planners/
INC_VALIDITY = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/validity_checkers/
INC_RUN = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/run/

EIGEN_DIR = /home/avishai/Documents/eigen

KDL_DIR = /usr/local

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${PQP_DIR}/include $(GL_INCPATH) -I${KDL_DIR}/include  -I$(EIGEN_DIR) 
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -L${KDL_DIR}/lib -lPQP -lm $(GL_LIBS) -larmadillo -lorocos-kdl
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp
CPPVRF = ${INC_VALIDITY}verification_class.cpp

CPPGDKDL = ${INC_CLASSES}gd_class.cpp ${INC_CLASSES}kdl_class.cpp 
CPPAPC = ${INC_CLASSES}apc_class.cpp ${INC_VALIDITY}StateValidityCheckerPCS.cpp
CPPGD = ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityCheckerGD.cpp

CPP_P_PCS = ${INC_RUN}plan_PCS.cpp ${INC_PLANNERS}CBiRRT_PCS.cpp ${INC_PLANNERS}RRT_PCS.cpp ${INC_PLANNERS}LazyRRT_PCS.cpp ${INC_PLANNERS}PRM_PCS.cpp ${INC_PLANNERS}SBL_PCS.cpp
CPP_P_PCS_s = ${INC_RUN}plan_PCS.cpp ${INC_PLANNERS}CBiRRT_PCS_s.cpp
CPP_P_GD = ${INC_RUN}plan_GD.cpp ${INC_PLANNERS}CBiRRT_GD.cpp ${INC_PLANNERS}RRT_GD.cpp ${INC_PLANNERS}LazyRRT_GD.cpp ${INC_PLANNERS}PRM_GD.cpp ${INC_PLANNERS}SBL_GD.cpp
CPP_P_GD_s = ${INC_RUN}plan_GD.cpp ${INC_PLANNERS}CBiRRT_GD_s.cpp

all:
	$(CXX) ${CPP_P_PCS} ${CPPAPC} ${CPPPQP} ${CPPVRF} -o ppcs $(CXXFLAGS) $(LDFLAGS) -std=c++11
	#$(CXX) ${CPP_P_PCS_s} ${CPPAPC} ${CPPPQP} ${CPPVRF} -o ppcss $(CXXFLAGS) $(LDFLAGS) -std=c++11

	#$(CXX) ${CPP_P_GD} ${CPPGD} ${CPPPQP} ${CPPVRF} -o pgd $(CXXFLAGS) $(LDFLAGS) -std=c++11
	#$(CXX) ${CPP_P_GD_s} ${CPPGD} ${CPPPQP} ${CPPVRF} -o pgds $(CXXFLAGS) $(LDFLAGS) -std=c++11




