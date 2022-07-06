#define PYTHON_IP_ADDRESS "127.0.0.1"
#define PYTHON_PORT_SENDING	5000
#define PYTHON_PORT_RECEIVING 5001

struct FromPythonPacket
{
    int effectType;
    float effectValue;
};

struct ToPythonPacket
{
    float coordInPipe;
};

#include "chai3d.h"

//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;
cHapticDeviceInfo hapticDeviceInfo;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few spherical objects
cVoxelObject* inertial_ball = nullptr;

UDPSocket* p_UDPSocketSending = nullptr;
UDPSocket* p_UDPSocketReceving = nullptr;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;
double verticalShift = 0.3;


const double PIPE_RADIUS = 0.005;
const double PIPE_HALF_LENGTH = 1.3;
cPrecisionClock myClock;

double workspaceScaleFactor;
double maxLinearForce;
double maxStiffness;
double maxDamping;

string resourceRoot;


double sign(double arg)
{
    if (arg>0)
        return 1;
    if (arg<0)
        return -1;
    return 0;
}

class NoEffectPositionGenerator
{
public:
    inline static cVector3d linVel = cVector3d(0,0,0);
    constexpr static double defaultNoEffectMass = 0.005;
    constexpr static double DAMPING = 0.1;
    inline static bool started = false;
    inline static double maxTimeInterval = 0.00477487;


    constexpr static const float angle = 30;

    virtual void setParameter(float paramVal = 0)
    {/*do nothing*/}
    virtual cVector3d generateSpecialForce()
    {
        return cVector3d(0,0,0);
    }
    virtual cVector3d getAcceleration(cVector3d force)
    {
        return (1.0 / defaultNoEffectMass) * force;
    }

    cVector3d rotateByAngle(const cVector3d& v, double angle)
    {
        cMatrix3d rotMat;
        rotMat = cMatrix3d(1,0,0,
                            0,1,0,
                            0,0,1);
        rotMat.setAxisAngleRotationDeg(cVector3d(0,0,1),angle);
        return cMul(rotMat, v);
    }

    virtual cVector3d generateBasicForce()
    {
        return -tool->getDeviceGlobalForce();
    }

    cVector3d getNextPos()
    {
        cVector3d force = this->generateBasicForce();
        if (!NoEffectPositionGenerator::started)
        {
            myClock.reset();
            myClock.start();
            NoEffectPositionGenerator::started = true;
        }
        force += this->generateSpecialForce();
        myClock.stop();
        double timeInterval = myClock.getCurrentTimeSeconds();
        if (timeInterval>maxTimeInterval) //safety block for low haptic frequency: used for preventing too long time intervals leading to overload of engines
            timeInterval = maxTimeInterval;
        myClock.reset();
        myClock.start();
        cVector3d linAcc = this->getAcceleration(force);
        NoEffectPositionGenerator::linVel.add(timeInterval * linAcc);
        NoEffectPositionGenerator::linVel.mul(1.0 - DAMPING * timeInterval);
        return inertial_ball->getLocalPos() + (timeInterval * NoEffectPositionGenerator::linVel);
    }

    cVector3d getNextPosLimitedByPipe(cVector3d nextPos)
    {
        cVector3d tempPos = rotateByAngle(nextPos, angle);
        cVector3d tempVel = rotateByAngle(NoEffectPositionGenerator::linVel, angle);
        double distance_from_center_proj = sqrt(cSqr(tempPos.x()) + cSqr(tempPos.z()));
        if (distance_from_center_proj > PIPE_RADIUS)
        {
            cVector3d limiting_vect(tempPos.x(), 0, tempPos.z());
            limiting_vect = limiting_vect/limiting_vect.length()*PIPE_RADIUS;
            tempPos = cVector3d(limiting_vect.x(),  tempPos.y(), limiting_vect.z());
            tempVel = cVector3d(0, tempVel.y(), 0);
        }

        if(abs(tempPos.y())>PIPE_HALF_LENGTH)
        {
            tempPos = cVector3d(tempPos.x(), sign(tempPos.y())*PIPE_HALF_LENGTH, tempPos.z());;
            tempVel = cVector3d(0, 0, 0);
        }
        NoEffectPositionGenerator::linVel = rotateByAngle(tempVel, -angle);
        nextPos = rotateByAngle(tempPos, -angle);
        return nextPos;
    }

    void updateToolPosition()
    {
        cVector3d nextPos = this->getNextPos();
        nextPos = this->getNextPosLimitedByPipe(nextPos);
        inertial_ball->setLocalPos(nextPos);
    }
};

class HoldPositionPositionGenerator:public NoEffectPositionGenerator
{
public:
    virtual cVector3d generateBasicForce()
    {
        NoEffectPositionGenerator::linVel *= 0;
        return cVector3d(0,0,0);
    }
};


class SpecifiedMassInertiaPositionGenerator:public NoEffectPositionGenerator
{
private:
    float mass = 0.3;
public:
    virtual void setParameter(float paramVal = 0)
    {
        this->mass = paramVal;
    }

    virtual cVector3d getAcceleration(cVector3d force)
    {
        return (1.0 / mass) * force;
    }
};

class DampingPositionGenerator:public NoEffectPositionGenerator
{
private:
    float dampingFactor = 0;

public:
    virtual void setParameter(float paramVal = 0)
    {
        this->dampingFactor = paramVal;
    }

    virtual cVector3d generateSpecialForce()
    {
        return -NoEffectPositionGenerator::linVel*dampingFactor;
    }
};

class NegativeSpringPositionGenerator:public NoEffectPositionGenerator
{
protected:
    float springCoefft = 0;
public:
    virtual void setParameter(float paramVal = 0)
    {
        this->springCoefft = paramVal;
    }

    virtual cVector3d generateSpecialForce() //odpychajaca od srodka
    {
        cVector3d currPos = inertial_ball->getLocalPos();
        if (currPos.length()>PIPE_HALF_LENGTH*0.9)// or abs(currPos.y())<0.1)
            return cVector3d(0,0,0);

        if (currPos.y()<0)
            return (cVector3d(0,-PIPE_HALF_LENGTH,0) - currPos)*springCoefft;
        else
            return (cVector3d(0,PIPE_HALF_LENGTH,0) - currPos)*springCoefft;
    }
};

class PositiveSpringPositionGenerator:public NegativeSpringPositionGenerator
{
    virtual cVector3d generateSpecialForce() //odpychajaca od srodka
    {
        cVector3d currPos = inertial_ball->getLocalPos();
        if (currPos.length()>PIPE_HALF_LENGTH*0.9)// or abs(currPos.y())<0.1)
            return cVector3d(0,0,0);
        return (cVector3d(0,0,0) - currPos)*this->springCoefft;
    }

};

class StickSlipPositionGenerator:public NoEffectPositionGenerator
{
private:
    cVector3d lastPoint;
    constexpr static double dx = 0.14;
    float StickSlipFactor = 0;
public:
    virtual void setParameter(float paramVal = 0)
    {
        this->StickSlipFactor = paramVal;
        lastPoint = cVector3d(0,0,0);
    }

    virtual cVector3d generateSpecialForce()
    {
        if (lastPoint.length()==0)
            lastPoint = inertial_ball->getLocalPos();
        cVector3d dist = inertial_ball->getLocalPos() - lastPoint;
        if (dist.length() > dx)
        {
            lastPoint = inertial_ball->getLocalPos();//lastPoint+dist/dist.length()*dx;
            dist = inertial_ball->getLocalPos() - lastPoint;
            return cVector3d(0,0,0);
        }
        return -dist*this->StickSlipFactor; //120 0.05 20 0.15
    }
};

class GoBackToStartGenerator:public NoEffectPositionGenerator
{
private:
    float dampingFactor = 1;
    int targetPosAchievedCounter = 0;
    double KiForceLim = 11;
    double Ki = 0.0006;
    double Kp = 4;
    cVector3d integralPart;
public:
    virtual void setParameter(float paramVal = 0)
    {
        this->integralPart = cVector3d(0,0,0);
    }

    virtual cVector3d generateSpecialForce()
    {
        cVector3d targetPos(0,PIPE_HALF_LENGTH*0.99, 0);
        targetPos = rotateByAngle(targetPos, -angle);
        cVector3d actPos = inertial_ball->getGlobalPos();
        cVector3d dist = targetPos - actPos;
        if(this->integralPart.length()*this->Ki<this->KiForceLim)
            this->integralPart += dist;
        cVector3d force = this->Kp*dist+this->Ki*this->integralPart;
        force -= NoEffectPositionGenerator::linVel*dampingFactor;
        return force;
    }
};

struct generatorOption
{
    string genName;
    NoEffectPositionGenerator* gen;
    float defaultVal;
};

map<char, generatorOption> generators;
NoEffectPositionGenerator* selectedGenerator;
void initGenerators()
{
    generators['c'] = {"No effect", new NoEffectPositionGenerator(), 0};
    generators['1'] = {"Inertial level 1", new SpecifiedMassInertiaPositionGenerator(), 0.05};
    generators['2'] = {"Liquid damping level 1", new DampingPositionGenerator(), 0.5};
    generators['3'] = {"Stick-slip level 1", new StickSlipPositionGenerator(), 15};
    generators['4'] = {"Negative spring level 1", new NegativeSpringPositionGenerator(), 1.3};
    generators['5'] = {"Positive spring level 1", new PositiveSpringPositionGenerator(), 1.5};

    generators['q'] = {"Inertia level 2", new SpecifiedMassInertiaPositionGenerator(), 0.5};
    generators['w'] = {"Liquid damping level 2", new DampingPositionGenerator(), 2};
    generators['e'] = {"Stick-slip level 2", new StickSlipPositionGenerator(), 40};
    generators['r'] = {"Negative spring level 2", new NegativeSpringPositionGenerator(), 3.5};
    generators['t'] = {"Positive spring level 2", new PositiveSpringPositionGenerator(), 4};

    generators['6'] = {"Hold position", new HoldPositionPositionGenerator(), 0};
    generators['7'] = {"Move back to start", new GoBackToStartGenerator(), 0};
    selectedGenerator = generators['c'].gen;
}


map<int, NoEffectPositionGenerator*> fromPythonGenerators;
void initFromPythonGenerators()
{
    fromPythonGenerators[1] = new NoEffectPositionGenerator();
    fromPythonGenerators[2] = new SpecifiedMassInertiaPositionGenerator();
    fromPythonGenerators[3] = new DampingPositionGenerator();
    fromPythonGenerators[4] = new StickSlipPositionGenerator();
    fromPythonGenerators[5] = new NegativeSpringPositionGenerator();
    fromPythonGenerators[6] = new PositiveSpringPositionGenerator();
    fromPythonGenerators[7] = new HoldPositionPositionGenerator();
    fromPythonGenerators[8] = new GoBackToStartGenerator();
    selectedGenerator = fromPythonGenerators[1];
}


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);

void buildSphere(cMultiImagePtr image, double radius, double thickness, int voxelModelResolution, cTexture3dPtr texture)
{
    cMutex mutexVoxel;
    mutexVoxel.acquire();
    double center = voxelModelResolution / 2.0;
    double radius_voxels = voxelModelResolution * radius;
    double thickness_voxels = voxelModelResolution * thickness;
    double k = 255.0 / voxelModelResolution;
    for (double z=0; z<voxelModelResolution; z+=1)
    {
        for (double y=0; y<voxelModelResolution; y+=1)
        {
            for (double x=0; x<voxelModelResolution; x+=1)
            {
                cColorb color;
                color.set(0x00, 0x00, 0x00, 0x00);
                double distance_from_center_proj = sqrt(cSqr(x-center) + cSqr(z-center) + cSqr(y-center));
                bool is_wall = abs(distance_from_center_proj - radius_voxels)<=thickness_voxels;
                if (is_wall)
                {
                    color.set(0xff, 0xff, 0xff, 0xff);
                    color.set((GLubyte)(k * fabs(x-center)),
                              (GLubyte)(k * fabs(y-center)),
                              (GLubyte)(k * fabs(z-center)), 0xff);
                }
                image->setVoxelColor(x, y, z, color);
            }
        }
    }
    texture->markForUpdate();
    mutexVoxel.release();
}

void createInertialSphereAroundTool(cWorld* &world)
{
    int voxelModelResolution = 255;
    inertial_ball = new cVoxelObject();
    world->addChild(inertial_ball);
    inertial_ball->m_minCorner.set(-1,-1,-1-verticalShift-0.05);
    inertial_ball->m_maxCorner.set(1,1,1-verticalShift-0.05);
    inertial_ball->m_minTextureCoord.set(0.0, 0.0, 0.0);
    inertial_ball->m_maxTextureCoord.set(1.0, 1.0, 1.0);
    inertial_ball->m_material->setOrangeCoral();
    inertial_ball->setStiffness(0.9* maxStiffness);
    inertial_ball->setShowBoundaryBox(false);
    cMultiImagePtr image = cMultiImage::create();
    image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
    cTexture3dPtr texture = cTexture3d::create();
    inertial_ball->setTexture(texture);
    texture->setImage(image);
    buildSphere(image, 0.015, 0.003, voxelModelResolution, texture);
    inertial_ball->setRenderingModeIsosurfaceMaterial();
    inertial_ball->createEffectSurface();
    inertial_ball->setQuality(0.2);
}

void initWorld(int argc, char* argv[])
{
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);
    glutInit(&argc, argv);
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");
    if (fullscreen)
    {
        glutFullScreen();
    }
    world = new cWorld();
    world->m_backgroundColor.setBlack();
}

void initCamera()
{
    camera = new cCamera(world);
    world->addChild(camera);

    camera->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    camera->setClippingPlanes(0.01, 10.0);
    camera->setUseMultipassTransparency(true);
    light = new cDirectionalLight(world);
    world->addChild(light);
    light->setEnabled(true);
    light->setDir(-1.0, -1.0, -1.0);
    light->setLocalPos(1.0, 1.0, 1.0);
}

void initHapticDeviceTool()
{
    handler = new cHapticDeviceHandler();
    handler->getDevice(hapticDevice, 0);
    hapticDeviceInfo = hapticDevice->getSpecifications();
    tool = new cToolCursor(world);
    world->addChild(tool);
    tool->setHapticDevice(hapticDevice);
    tool->setRadius(0.015);
    tool->setWorkspaceRadius(1.0);
    //tool->setWaitForSmallForce(true);
    tool->enableDynamicObjects(true);
    tool->start();

    workspaceScaleFactor = tool->getWorkspaceScaleFactor();
    maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;
}


void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

void close(void)
{
    simulationRunning = false;
    while (!simulationFinished)
    {
        cSleepMs(100);
    }
    tool->stop();
}

void graphicsTimer(int data)
{
    if (simulationRunning)
        glutPostRedisplay();
    glutTimerFunc(50, graphicsTimer, 0);
}

void configUDP()
{
    p_UDPSocketSending = new UDPSocket();
    p_UDPSocketSending->configureNetwork(PYTHON_PORT_SENDING, PYTHON_IP_ADDRESS, PYTHON_PORT_SENDING);
    p_UDPSocketReceving = new UDPSocket();
    p_UDPSocketReceving->configureNetwork(PYTHON_PORT_RECEIVING, PYTHON_IP_ADDRESS, PYTHON_PORT_RECEIVING);
}

bool openedReceiving = false;
void receiveFromPython()
{
    if (!openedReceiving)
    {
        p_UDPSocketReceving->open();
        openedReceiving = true;
    }
    FromPythonPacket fpp;
    int bytesCnt = p_UDPSocketReceving->receiveData(&fpp, sizeof(fpp));
    if (bytesCnt==sizeof(fpp))
    {
        cout << fpp.effectType<<" "<<fpp.effectValue<<endl;

        if (fromPythonGenerators.find(fpp.effectType)!=fromPythonGenerators.end())
        {
            NoEffectPositionGenerator* effect = fromPythonGenerators[fpp.effectType];
            effect->setParameter(fpp.effectValue);
            effect->setParameter(fpp.effectValue);
            selectedGenerator = effect;
        }
    }
}

bool openedSending = false;

void sendPositionToPython()
{
    if (!openedSending)
    {
        //p_UDPSocketSending->open();
        p_UDPSocketSending->createSocketWithoutPortBlocking();
        openedSending = true;
    }

    cVector3d pos = inertial_ball->getLocalPos();
    ToPythonPacket tpp = {pos.length()*pos.y()};
    p_UDPSocketSending->sendData(&tpp, sizeof(tpp));
}

void updateGraphics(void)
{
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);
    world->updateShadowMaps(false, mirroredDisplay);
    camera->renderView(windowW, windowH);
    glutSwapBuffers();
    glFinish();
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
    sendPositionToPython();
    receiveFromPython();

}


void keySelect(unsigned char key, int x, int y)
{
    if ((key == 27) || (key == 'x'))
    {
        exit(0);
    }
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    if (generators.find(key) != generators.end())
    {
        generatorOption go = generators[key];
        go.gen->setParameter(go.defaultVal);
        selectedGenerator = go.gen;
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

int main(int argc, char* argv[])
{
    configUDP();
    initWorld(argc, argv);
    initCamera();

    initHapticDeviceTool();
    initGenerators();
    initFromPythonGenerators();
    map<char, generatorOption>::iterator it = generators.begin();
    while(it!=generators.end())
    {
        cout << "["<<  it->first<<"] - "<<it->second.genName<<endl;
        it  = next(it);
    }
    configUDP();


    createInertialSphereAroundTool(world);

    cFont *font = NEW_CFONTCALIBRI20();

    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    atexit(close);
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    return (0);
}

void updateHaptics(void)
{
    simulationRunning  = true;
    simulationFinished = false;
    while(simulationRunning)
    {
        frequencyCounter.signal(1);
        selectedGenerator->updateToolPosition();
        world->computeGlobalPositions(true);
        tool->updateFromDevice();
        tool->computeInteractionForces();
        tool->applyToDevice();
    }
    simulationFinished = true;
}
