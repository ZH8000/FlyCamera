#include <iostream>
#include <Python.h>

using namespace std;

int main(int argc, char* argv[]) {
    cout << "run GPIO............." << endl;

    Py_Initialize();

    PyObject* pSignalString = PyString_FromString((char*)"signal");
    PyObject* pSignal = PyImport_Import(pSignalString); // import signal;

    PyObject* pTimeString = PyString_FromString((char*)"time");
    PyObject* pTime = PyImport_Import(pTimeString); // import time;
    PyObject* pSleepFunc = PyObject_GetAttrString(pTime, (char*)"sleep"); // time.sleep(...);
    PyObject* pSleepFuncArgs = PyTuple_Pack(1, PyInt_FromLong(1)); // (1)

    PyObject* pGPIOString = PyString_FromString((char*)"Adafruit_GPIO");
    PyObject* pGPIO = PyImport_Import(pGPIOString); // import Adafruit_GPIO

    PyObject* pFT232HString = PyString_FromString((char*)"Adafruit_GPIO.FT232H");
    PyObject* pFT232H = PyImport_Import(pFT232HString); // import Adafruit_GPIO.FT232H

    PyObject* pSignalFunc = PyObject_GetAttrString(pSignal, (char*)"signal"); // signal.signal(...);
    PyObject* pSIGINT = PyObject_GetAttrString(pSignal, (char*)"SIGINT"); // signal.SIGINT
    PyObject* pSIG_DFL = PyObject_GetAttrString(pSignal, (char*)"SIG_DFL"); // signal.SIG_DFL
    PyObject* pSignalFuncArgs = PyTuple_Pack(2, pSIGINT, pSIG_DFL); // (signal.SIGINT, signal.SIG_DFL)
    PyObject* pSignalCall = PyObject_CallObject(pSignalFunc, pSignalFuncArgs);

    PyObject* pUseFT232H = PyObject_GetAttrString(pFT232H, (char*)"use_FT232H"); // FT232H.use_FT232H();
    PyObject* pft232h = PyObject_GetAttrString(pFT232H, (char*)"FT232H"); // ft232h = FT232H.FT232H;

    PyObject* pGPIOin = PyObject_GetAttrString(pGPIO, (char*)"IN"); // Adafruit_GPIO.IN
    PyObject* pGPIOout = PyObject_GetAttrString(pGPIO, (char*)"OUT"); // Adafruit_GPIO.OUT
    PyObject* pGPIOhigh = PyObject_GetAttrString(pGPIO, (char*)"HIGH"); // Adafruit_GPIO.HIGH
    PyObject* pGPIOlow = PyObject_GetAttrString(pGPIO, (char*)"LOW"); // Adafruit_GPIO.LOW

    PyObject* pSetupFunc = PyObject_GetAttrString(pft232h, (char*)"setup"); // ft232h.setup(...);
    PyObject* pD7SetupFuncArgs = PyTuple_Pack(2, PyInt_FromLong(7), pGPIOin); // (7, GPIO.IN); // D7
    PyObject* pC0SetupFuncArgs = PyTuple_Pack(2, PyInt_FromLong(8), pGPIOout); // (8, GPIO.OUT); // C0
    PyObject* pD7Setup = PyObject_CallObject(pSetupFunc, pD7SetupFuncArgs); // ft232h.setup(7, GPIO.IN); // D7 
    PyObject* pC0Setup = PyObject_CallObject(pSetupFunc, pC0SetupFuncArgs); // ft232h.setup(8, GPIO.OUT); // C0
    PyObject* pOutput = PyObject_GetAttrString(pft232h, (char*)"output");

    

    while(1) {
        PyObject* pC0HighArgs = PyTuple_Pack(2, PyInt_FromLong(8), pGPIOhigh); // (8, GPIO.HIGH)
        PyObject* pC0HighFunc = PyObject_CallObject(pOutput, pC0HighArgs); // ft232h.output(8, GPIO.HIGH)

        PyObject* pSleepCall = PyObject_CallObject(pSleepFunc, pSleepFuncArgs);

        PyObject* pC0LowArgs = PyTuple_Pack(2, PyInt_FromLong(8), pGPIOlow); // (8, GPIO.LOW)
        PyObject* pC0LowFunc = PyObject_CallObject(pOutput, pC0LowArgs); // ft232h.output(8, GPIO.LOW)
        cout << "whiling..." << endl;
    }

    cout << "run GPIO end." << endl;
/*
    PyObject *pModuleGPIO, *pModuleFT232H;
    
    // Initialize the Python Interpreter
    Py_Initialize();

    try {
        PyRun_SimpleString("import signal");
        PyRun_SimpleString("import time");
        PyRun_SimpleString("import Adafruit_GPIO as GPIO");
        PyRun_SimpleString("import Adafruit_GPIO.FT232H as FT232H");

        PyRun_SimpleString("signal.signal(signal.SIGINT, signal.SIG_DFL)");

        PyRun_SimpleString("FT232H.use_FT232H()");
        PyRun_SimpleString("ft232h = FT232H.FT232H()");

        PyRun_SimpleString("ft232h.setup(7, GPIO.IN)");   // Make pin D7 a digital input.
        PyRun_SimpleString("ft232h.setup(8, GPIO.OUT)");  // Make pin C0 a digital input.
        PyRun_SimpleString("ft232h.setup(9, GPIO.OUT)");  // Make pin C1 a digital input.
        PyRun_SimpleString("ft232h.setup(10, GPIO.OUT)"); // Make pin C2 a digital input.
        PyRun_SimpleString("ft232h.setup(11, GPIO.OUT)"); // Make pin C3 a digital input.

        PyRun_SimpleString("print 'Press Ctrl-C to quit.'");
        while ( true) {
        //PyRun_SimpleString("while True:");
            PyRun_SimpleString("ft232h.output(8, GPIO.HIGH)");
            PyRun_SimpleString("ft232h.output(9, GPIO.HIGH)");
            PyRun_SimpleString("ft232h.output(10, GPIO.HIGH)");
            PyRun_SimpleString("ft232h.output(11, GPIO.HIGH)");
            PyRun_SimpleString("time.sleep(1)");
            PyRun_SimpleString("ft232h.output(8, GPIO.LOW)");
            PyRun_SimpleString("ft232h.output(9, GPIO.LOW)");
            PyRun_SimpleString("ft232h.output(10, GPIO.LOW)");
            PyRun_SimpleString("ft232h.output(11, GPIO.LOW)");
            PyRun_SimpleString("time.sleep(1)");
            PyRun_SimpleString("level = ft232h.input(7)");
            PyRun_SimpleString("level = ft232h.input(7)");

        }
    } catch(...) {
        PyErr_Print();
        PyErr_Clear();
        return false;
    }
*/
}
