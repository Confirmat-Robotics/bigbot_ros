#ifndef ROTATE_HPP
#define ROTATE_HPP

class Rotate {
public:
    Rotate(double angle, double radius, bool forward = true, bool inverse_direction = false)
        : angle_(inverse_direction ? -angle : angle), radius_(radius), forward_(forward) {}

    double getAngle() const { return angle_; }
    double getRadius() const { return radius_; }
    bool isForward() const { return forward_; }
    
    // Virtual destructor to enable polymorphism
    virtual ~Rotate() {}

private:
    double angle_;
    double radius_;
    bool forward_;
};

#endif // ROTATE_HPP

