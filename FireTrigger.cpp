#include <Python.h>

int main(int argc, char* argv[]) {
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

        PyRun_SimpleString("ft232h.setup(9, GPIO.OUT)");   // Make pin D6 a digital input.
        /*
        PyRun_SimpleString("ft232h.setup(7, GPIO.IN)");   // Make pin D7 a digital input.
        PyRun_SimpleString("ft232h.setup(8, GPIO.OUT)");  // Make pin C0 a digital input.
        PyRun_SimpleString("ft232h.setup(9, GPIO.OUT)");  // Make pin C1 a digital input.
        PyRun_SimpleString("ft232h.setup(10, GPIO.OUT)"); // Make pin C2 a digital input.
        //PyRun_SimpleString("ft232h.setup(11, GPIO.OUT)"); // Make pin C3 a digital input.
        */

        PyRun_SimpleString("print 'Press Ctrl-C to quit.'");
        while ( true) {
        //PyRun_SimpleString("while True:");
            PyRun_SimpleString("ft232h.output(9, GPIO.HIGH)");
            PyRun_SimpleString("ft232h.output(9, GPIO.LOW)");
            /*
            PyRun_SimpleString("ft232h.output(8, GPIO.HIGH)");
            PyRun_SimpleString("ft232h.output(9, GPIO.HIGH)");
            PyRun_SimpleString("ft232h.output(10, GPIO.HIGH)");
            //PyRun_SimpleString("ft232h.output(11, GPIO.HIGH)");
            PyRun_SimpleString("time.sleep(1)");
            PyRun_SimpleString("ft232h.output(6, GPIO.LOW)");
            PyRun_SimpleString("ft232h.output(8, GPIO.LOW)");
            PyRun_SimpleString("ft232h.output(9, GPIO.LOW)");
            PyRun_SimpleString("ft232h.output(10, GPIO.LOW)");
            //PyRun_SimpleString("ft232h.output(11, GPIO.LOW)");
            PyRun_SimpleString("time.sleep(1)");
            PyRun_SimpleString("level = ft232h.input(7)");
            */
        }
    } catch(...) {
        PyErr_Print();
        PyErr_Clear();
        return false;
    }
}
