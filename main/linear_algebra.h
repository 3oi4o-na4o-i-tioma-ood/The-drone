#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_
#include <math.h>
#include <stdint.h>

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct
{
    float x;
    float y;
    float z;
} VectorFloat;

Quaternion getConjugate(const Quaternion *q)
{
    Quaternion res = {
        q->w,
        -q->x,
        -q->y,
        -q->z,
    };

    return res;
}

Quaternion getProduct(const Quaternion *q1, const Quaternion *q2)
{
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    Quaternion res =  {
        q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z, // new w
        q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y, // new x
        q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x, // new y
        q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w  // new z
    };

    return res;
}

void rotateWithQuaternion(VectorFloat *vector, const Quaternion *q)
{
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
    // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector (a*aReal)
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
    Quaternion p = {0, vector->x, vector->y, vector->z};

    // quaternion multiplication: q * p, stored back in p
    p = getProduct(q, &p);

    Quaternion conjugate = getConjugate(q);

    // quaternion multiplication: p * conj(q), stored back in p
    p = getProduct(&p, &conjugate);

    // p quaternion is now [0, x', y', z']
    vector->x = p.x;
    vector->y = p.y;
    vector->z = p.z;
}



// float getMagnitude()
// {
//     return sqrt(w * w + x * x + y * y + z * z);
// }

// void normalize()
// {
//     float m = getMagnitude();
//     w /= m;
//     x /= m;
//     y /= m;
//     z /= m;
// }

// Quaternion getNormalized()
// {
//     Quaternion r(w, x, y, z);
//     r.normalize();
//     return r;
// }

// class VectorFloat
// {
// public:
//     float x;
//     float y;
//     float z;

//     VectorFloat()
//     {
//         x = 0;
//         y = 0;
//         z = 0;
//     }

//     VectorFloat(float nx, float ny, float nz)
//     {
//         x = nx;
//         y = ny;
//         z = nz;
//     }

//     float getMagnitude()
//     {
//         return sqrt(x * x + y * y + z * z);
//     }

//     void normalize()
//     {
//         float m = getMagnitude();
//         x /= m;
//         y /= m;
//         z /= m;
//     }

//     VectorFloat getNormalized()
//     {
//         VectorFloat r(x, y, z);
//         r.normalize();
//         return r;
//     }

//     void rotate(Quaternion *q)
//     {
//         Quaternion p(0, x, y, z);

//         // quaternion multiplication: q * p, stored back in p
//         p = q->getProduct(p);

//         // quaternion multiplication: p * conj(q), stored back in p
//         p = p.getProduct(q->getConjugate());

//         // p quaternion is now [0, x', y', z']
//         x = p.x;
//         y = p.y;
//         z = p.z;
//     }

//     VectorFloat getRotated(Quaternion *q)
//     {
//         VectorFloat r(x, y, z);
//         r.rotate(q);
//         return r;
//     }
// };

#endif /* _HELPER_3DMATH_H_ */