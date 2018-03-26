#ifndef LABELING_H
#define LABELING_H

class Label {
public:
    static const int GROUND = 1;
    static const int NONE = 0;
};

typedef enum {
    MOVE,
    MARK_GROUND,
    CORRECTION
} AnnotationContext;

#endif // LABELING_H
