#ifndef CONTROL_MODE_H
#define CONTROL_MODE_H

class ControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~ControlMode() = default;

    // Pure virtual public functions
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual void update() = 0;

protected:
    // Protected constructor to prevent direct instantiation
    ControlMode() = default;

    // Protected setup function for derived classes
    virtual bool setupControl() = 0;

private:
    // Private member variables can be added here
    bool is_running_ = false;
};

#endif // CONTROL_MODE_H