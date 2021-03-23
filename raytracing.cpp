#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define M_PI 3.14159265

#include <random>
static std::default_random_engine engine(10); // random seed=10
static std::uniform_real_distribution<double> uniform(1.,0);

class Vector {
    public:
        explicit Vector(double x=0, double y=0, double z=0) {
            coords[0] = x;
            coords[1] = y;
            coords[2] = z;
        };
        double operator[](int i) const { return coords[i]; };
        double &operator[](int i) { return coords[i]; };
        double sqrNorm() const {
            return coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2];
        }
        Vector get_normalized() {
            double n = sqrt(sqrNorm());
            return Vector(coords[0] / n, coords[1] / n, coords[2] / n);
        }
    private:
        double coords[3];
};

// Surcharge d'opérateur
Vector operator+(const Vector &a, const Vector &b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector &a, const Vector &b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator- (const Vector &a) {
    return(Vector(-a[0], -a[1], -a[2]));
};
Vector operator*(double a, const Vector &b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector &a, double b) {
    return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(const Vector &a, const Vector &b) {
    return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
Vector operator/(const Vector &a, double b) {
    return Vector(a[0]/b, a[1]/b, a[2]/b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector &a, const Vector &b){
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
double sqr(double x){
    return x*x;
}

// Random cos
Vector random_cos(const Vector &N){
    double u1 = uniform(engine);
    double u2 = uniform(engine);
    double x = cos(2 * M_PI * u1) * sqrt(1 - u2);
    double y = sin(2 * M_PI * u1) * sqrt(1 - u2);
    double z = sqrt(u2);

    Vector T1;

    if(N[0] < N[1] && N[0] < N[2]){
        T1 = Vector(0, N[2], -N[1]);
    }
    else{
        if(N[1] < N[2] && N[1] < N[0]){
            T1 = Vector(N[2], 0, -N[0]);
        }
        else{
            T1 = Vector(N[1], -N[0],0);
        }
    }

    T1 = T1.get_normalized();
    Vector T2 = cross(N,T1);

    return x*T1 + y*T2 + z*N;
}


class Ray {
    public: Ray(const Vector &C, const Vector &u) : C(C), u(u) {
    }
    Vector C, u;
};

class Object{
public:
    Object(){};
    virtual bool intersect(const Ray &r, Vector &P, Vector &normale, double &t, Vector &color) = 0;

    Vector albedo;
    bool isMirror, isTransparent;
};

class Sphere : public Object {
    public:
        Sphere(const Vector& O, double R, const Vector &albedo, bool isMirror = false, bool isTransparent = false): O(O), R(R) {
            this->albedo = albedo;
            this->isMirror = isMirror;
            this->isTransparent = isTransparent;
        }
        Vector O;
        double R;
        bool intersect(const Ray &r, Vector &P, Vector &N, double &t, Vector &color) {
            double a = 1;
            double b = 2 * dot(r.u, r.C - O);
            double c = (r.C-O).sqrNorm() - R*R;
            double delta = b*b - 4*a*c;
            if (delta < 0) return false;


            double sqrtDelta = sqrt(delta);

            double t1 = (-b - sqrtDelta) / (2 * a);
            double t2 = (-b + sqrtDelta) / (2 * a);

            if (t2 < 0) return false;

            if (t1 > 0)
                t = t1;
            else
                t = t2;

            P = r.C + t*r.u;
            N = (P-O).get_normalized();

            color = this ->albedo;
            return true;
        };
};

class BoundingBox {
public:
    bool intersect(const Ray& r){
        double tx1 = (mini[0] - r.C[0]) / r.u[0];
        double tx2 = (maxi[0] - r.C[0]) / r.u[0];
        double txmin = std::min(tx1, tx2), txmax = std::max(tx1, tx2);

        double ty1 = (mini[1] - r.C[1]) / r.u[1];
        double ty2 = (maxi[1] - r.C[1]) / r.u[1];
        double tymin = std::min(ty1, ty2), tymax = std::max(ty1, ty2);

        double tz1 = (mini[2] - r.C[2]) / r.u[2];
        double tz2 = (maxi[2] - r.C[2]) / r.u[2];
        double tzmin = std::min(tz1, tz2), tzmax = std::max(tz1, tz2);

        double tMax = std::min(txmax,std::min(tymax,tzmax));
        double tMin = std::max(txmin,std::min(tymin,tzmin));

        if (tMax <0) return false;
        return tMax > tMin;

    }
    Vector mini, maxi;
};

class Noeud {
public:
        Noeud *fg, *fd;
        BoundingBox b;
        int start, end;
};

class Scene {
public:
    Scene(){};
    std::vector<Object*> objects;
    Vector L;
    double I;

    bool intersect(const Ray& r, Vector& P, Vector& N, Vector &albedo, bool &mirror, bool &transp, double &t, int &objectID) {
        t = 1E10;
        bool is_inter = false;

        for(int i=0; i < objects.size(); i++) {
            Vector localP, localN, localAlbedo;
            double localT;

            if(objects[i]->intersect(r, localP, localN, localT, localAlbedo) && localT < t) {
                t = localT;
                is_inter = true;
                albedo = localAlbedo;
                P = localP;
                N = localN;
                mirror = objects[i]->isMirror;
                transp = objects[i]->isTransparent;
                objectID = 1;
            };
        };
        return is_inter;
    };

    Vector getColor(const Ray &r, int rebond, bool lastdiffuse) {

        double eps = 0.00001;

        Vector P, N, albedo;
        double t;
        bool mirror, transp;
        int objectID;
        bool inter = intersect(r, P, N, albedo, mirror, transp, t, objectID);
        Vector color(0,0,0);

        if(rebond > 5) {
            return Vector(0.,0.,0.);
        }

        if(inter){
            if(objectID == 0){
                if (rebond == 0 || !lastdiffuse) {
                    return Vector(I, I, I) / (4 * M_PI * M_PI * sqr(dynamic_cast<Sphere*>(objects[0])->R));
                }
                else{
                    return Vector(0, 0, 0);
                }
            }

            else{
                // Miroir
                if (mirror){
                    Vector reflectedDir = r.u - 2 * dot(r.u, N) * N;
                    Ray reflectedRay(P + eps * N, reflectedDir);

                    return getColor(reflectedRay, rebond + 1, false);
                }

                else{
                    // Transparent
                    if (transp){

                        double n1 = 1, n2 = 1.4;  // 1.4 indice refract verre
                        Vector N2 = N;

                        if(dot(r.u, N) > 0){ // en quittant la sphere, on inverse les indices
                            std::swap(n1, n2);
                            N2 = -N;
                        };

                        double rad = 1 - sqr(n1 / n2) * (1 - sqr(dot(r.u,N2)));

                        if(rad < 0){ // 100% reflexion, 0 transmission
                            Vector reflectedDir = r.u - 2 * dot(r.u, N) * N;
                            Ray reflectedRay(P + eps * N, reflectedDir);
                            return getColor(reflectedRay, rebond + 1, false);
                        }


                        Vector Tt = n1/n2 * (r.u - dot(r.u, N2) * N2);
                        Vector Tn = -sqrt(rad) * N2;

                        Vector refractedDir = Tt + Tn;
                        Ray refractedRay(P - eps * N2, refractedDir);

                        return getColor(refractedRay, rebond + 1, false);
                    }

                    // Eclairage direct
                    else{
                        Vector PL = L - P;
                        PL = PL.get_normalized();
                        Vector w = random_cos(-PL);

                        Vector origin = dynamic_cast<Sphere*>(objects[0])->O;
                        double radius = dynamic_cast<Sphere*>(objects[0])->R;

                        Vector xprime = w * radius + origin;
                        Vector Pxprime = xprime - P;
                        double d = sqrt(Pxprime.sqrNorm());
                        Pxprime = Pxprime / d;

                        Vector shadowP, shadowN, shadowAlbedo;
                        double shadowt;
                        int shadowID;
                        bool shadowMirror, shadowTransp;

                        Ray shadowRay(P + eps * N, Pxprime);

                        bool shadowInter = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowMirror, shadowTransp, shadowt, shadowID);

                        if(shadowInter && shadowt < d - eps){
                            color = Vector(0.,0.,0.);
                        }
                        else{
                            double proba = std::max(0.,dot(-PL,w)) / (M_PI * radius * radius);
                            double J = std::max(0.,dot(w, -Pxprime) / (d*d));
                            color =  I / (4 * M_PI * M_PI * sqr(radius)) * (albedo / M_PI) * std::max(0., dot(N, Pxprime)) * J / proba;
                        }

                        // Eclairage indirect
                        Vector vect_ind = random_cos(N); // Reflected Ray Direction
                        Ray indRay(P + eps * N, vect_ind.get_normalized());
                        color = color + albedo / M_PI * getColor(indRay, rebond + 1, true);
                    }
                }
            }
        }
        return color;
    };
};

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk;
	int uvi, uvj, uvk;
	int ni, nj, nk;
	int group;
};


int main() {
    int W = 256;
    int H = 256;
    int nb_rays = 10;

    Vector C(0,0,55);
    Scene scene;
    scene.I = 5E9;
    scene.L = Vector(-10, 20, 40);
    double angle = 60;
    double alpha = angle * M_PI / 180;

    // Objets de la scène
    Sphere Lumiere(scene.L, 5, Vector(1.,1.,1.));
    Sphere S1(Vector(0, 0, 0), 7, Vector(1, 1, 1), false, false);
    Sphere S2(Vector(-15, 10, 0), 7, Vector(1., 1., 0.5), true, false);
    Sphere S3(Vector(15, 10, 0), 7, Vector(1., 0.5, 1.), false, true);
    Sphere Sol(Vector(0, -1000, 0),990,Vector(0.5, 1., 0.5));
    Sphere Plafond(Vector(0, 1000, 0),940,Vector(1., 0., 0.));
    Sphere MurG(Vector(-1000, 0 ,0),960,Vector(1., 0.5, 0.5));
    Sphere MurD(Vector(1000, 0, 0),960,Vector(0., 0., 1.));
    Sphere MurDer(Vector(0, 0, -1000),940,Vector(1., 0.2, 0.2));
    Sphere MurDev(Vector(0, 0, 1000),940,Vector(1., 1., 0.5));

    scene.objects.push_back(&Lumiere);
    scene.objects.push_back(&S1);
    scene.objects.push_back(&S2);
    scene.objects.push_back(&S3);
    scene.objects.push_back(&MurG);
    scene.objects.push_back(&MurD);
    scene.objects.push_back(&MurDer);
    scene.objects.push_back(&MurDev);
    scene.objects.push_back(&Plafond);
    scene.objects.push_back(&Sol);

    // Profondeur de champ
    double cpsz = 0.001;
    double focale = 55;

    std::vector<unsigned char> image(W * H * 3, 0);
    #pragma omp parallel for schedule(dynamic,1)
    for (int i = 0; i < H; i++) { // Height
        for (int j = 0; j < W; j++) { // Width

            Vector color(0,0,0);

            for(int k=0; k < nb_rays; k++){ // Rays

                double u1 = uniform(engine);
                double u2 = uniform(engine);
                double dx = 0.25 * cos(2 * M_PI * u1) * sqrt(-2 * log(u2));
                double dy = 0.25 * sin(2 * M_PI * u1) * sqrt(-2 * log(u2));

                u1 = uniform(engine);
                u2 = uniform(engine);
                double dx2 = cpsz * cos(2 * M_PI * u1) * sqrt(-2 * log(u2));
                double dy2 = cpsz * sin(2 * M_PI * u1) * sqrt(-2 * log(u2));

                Vector dir(j - W/2 + dx +0.5, i - H/2 + dy + 0.5, -W / (2. * tan(alpha/2))); // Inverser dx1 et dx2 ?
                dir = dir.get_normalized();

                Vector target = C + focale * dir;
                Vector C_prime = C + Vector(dx2, dy2, 0);
                Vector dir_prime = (target - C_prime).get_normalized();

                Ray r(C_prime,dir_prime);

                color = color + scene.getColor(r, 0, false);
            };
            color = color / nb_rays;

            image[((H - i - 1) * W + j) * 3 + 0] = std::min(255., std::pow(color[0],0.45));
            image[((H - i - 1) * W + j) * 3 + 1] = std::min(255., std::pow(color[1],0.45));
            image[((H - i - 1) * W + j) * 3 + 2] = std::min(255., std::pow(color[2],0.45));
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);

    return 0;
}
