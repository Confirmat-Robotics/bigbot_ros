#ifndef TRANSLATE_HPP
#define TRANSLATE_HPP

class Translate {
public:
    Translate(double length, bool forward = true)
        : length_(length), forward_(forward) {}

    double getLength() const { return length_; }
    bool isForward() const { return forward_; }

    // Virtual destructor to enable polymorphism
    virtual ~Translate() {}
    
private:
    double length_;
    bool forward_;
};

#endif // TRANSLATE_HPP

