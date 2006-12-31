#include <iostream>
#include <algorithm>
#include <SDL.h>

using std::max;
using std::pair;
using std::swap;
using std::cout;
using std::cerr;
using std::endl;

const int WINDOW_WIDTH = 1440 / 2;
const int WINDOW_HEIGHT = 900 / 2;

const int MAX_CNT = 100;

enum ClPointType { LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGN, DEST };
enum IntersectType { COLLINEAR, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS };
enum EType { TOUCHING, CROSS_LEFT, CROSS_RIGHT, INESSENTIAL };
enum PType { INSIDE, OUTSIDE };				// Внутри, снаружи
enum PolygonType1 { SIMPLE, COMPLEX };		// Простой (без самопересечений), сложный
enum PolygonType2 { CONVEX, CONCAVE };		// Выпуклый, невыпуклый
enum LineCapsType { FLAT, ROUND, SQUARE };	// Плоские, круглые, квадратные

struct Point
{
	int x, y;
};

bool operator == (const Point p1, const Point p2)
{
	return p1.x == p2.x && p1.y == p2.y;
}

struct Hatch
{
	int count;
	unsigned char h[MAX_CNT];
};

class Matrix
{
	double m[4][4];

	void check_index(int i, int j) const
	{
		if (i < 0 || j < 0 || i >= 4 || j >= 4)
		{
			cerr << "Error: Wrong matrix index (" << i << "," << j << ")" << endl;
			exit(1);
		}
	}

public:
	Matrix()
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				m[i][j] = 0.0;
	}

	double operator () (int i, int j) const { check_index(i,j); return m[i][j]; }
	double & operator () (int i, int j)     { check_index(i,j); return m[i][j]; }

	Matrix operator * (const Matrix &mx) const
	{
		Matrix res;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				for (int k = 0; k < 4; k++)
					res.m[i][j] += m[i][k] * mx.m[k][j];
		return res;
	}

	Matrix & operator *= (const Matrix &mx) { *this = *this * mx; return *this; }

	static const Matrix Diagonal()
	{
		Matrix res;
		res.m[0][0] = 1.0;
		res.m[1][1] = 1.0;
		res.m[2][2] = 1.0;
		res.m[3][3] = 1.0;
		return res;
	}
};

class Vector4
{
public:
	double x, y, z, w;

	Vector4(): x(0.0), y(0.0), z(0.0), w(1.0) { }
	Vector4(double X, double Y, double Z): x(X), y(Y), z(Z), w(1.0) { }
	Vector4(double X, double Y, double Z, double W): x(X), y(Y), z(Z), w(W) { }

	Vector4 operator * (const Matrix &m) const
	{
		Vector4 r;
		r.x = x * m(0,0) + y * m(1,0) + z * m(2,0) + w * m(3,0);
		r.y = x * m(0,1) + y * m(1,1) + z * m(2,1) + w * m(3,1);
		r.z = x * m(0,2) + y * m(1,2) + z * m(2,2) + w * m(3,2);
		r.w = x * m(0,3) + y * m(1,3) + z * m(2,3) + w * m(3,3);

		r.x /= r.w;
		r.y /= r.w;
		r.z /= r.w;
		r.w = 1.0;
		return r;
	}

	Vector4 & operator *= (const Matrix &m) { *this = *this * m; return *this; }
};

class Vector3
{
public:
	double x, y, z;

	Vector3(): x(0.0), y(0.0), z(0.0) { }
	Vector3(double X, double Y, double Z): x(X), y(Y), z(Z) { }
	Vector3(const Vector4 v4): x(v4.x / v4.w), y(v4.y / v4.w), z(v4.z / v4.w) { }
	double Length() const { return sqrt(x*x + y*y + z*z); }
	void Normalize() { *this /= Length(); }
	Vector3 operator - () const { return Vector3(-x, -y, -z); }
	Vector3 operator + (const Vector3 v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
	Vector3 operator - (const Vector3 v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
	double operator * (const Vector3 v) const { return x * v.x + y * v.y + z * v.z; }
	Vector3 operator / (double k) const { return Vector3(x / k, y / k, z / k); }
	Vector3 & operator /= (double k) { x /= k; y /= k; z /= k; return *this; }
	Vector3 operator ^ (const Vector3 v) const
	{
		return Vector3(y * v.z - z * v.y,
					   z * v.x - x * v.z,
					   x * v.y - y * v.x);
	}
};

class Polygon
{
	int p_count;
	Point p[MAX_CNT];

	void check_index(int i) const
	{
		if (i < 0 || i >= p_count)
		{
			cerr << "Error: Wrong polygon index (" << i << ")" << endl;
			exit(1);
		}
	}

public:
	Polygon(int count = 0): p_count(count) { }
	int count() const { return p_count; }
	Point operator [] (int i) const	{ check_index(i); return p[i]; }
	Point & operator [] (int i)		{ check_index(i); return p[i]; }
	void operator += (Point new_p)	{ p[p_count++] = new_p; }
};

class RPP	// Прямоугольный параллелепипед
{
public:
	Vector4 a1, b1, c1, d1,
			a2, b2, c2, d2;

	RPP(double x, double y, double z)
	{
		a1 = Vector4(-x, -y,  z);
		b1 = Vector4(-x,  y,  z);
		c1 = Vector4(-x,  y, -z);
		d1 = Vector4(-x, -y, -z);
		a2 = Vector4( x, -y,  z);
		b2 = Vector4( x,  y,  z);
		c2 = Vector4( x,  y, -z);
		d2 = Vector4( x, -y, -z);
	}

	void operator *= (const Matrix &m)
	{
		a1 *= m;  b1 *= m;  c1 *= m;  d1 *= m;
		a2 *= m;  b2 *= m;  c2 *= m;  d2 *= m;
	}
};

class RGBColor
{
public:
	uint8_t r, g, b;

	RGBColor(uint8_t red, uint8_t green, uint8_t blue)
		: r(red), g(green), b(blue) { }

	static const RGBColor White()	{ return RGBColor(0xFF, 0xFF, 0xFF); }
	static const RGBColor Red()		{ return RGBColor(0xFF, 0x00, 0x00); }
	static const RGBColor Green()	{ return RGBColor(0x00, 0xFF, 0x00); }
	static const RGBColor Blue()	{ return RGBColor(0x00, 0x00, 0xFF); }
	static const RGBColor Magenta()	{ return RGBColor(0xFF, 0x00, 0xFF); }
	static const RGBColor Acid()	{ return RGBColor(0xC6, 0xFF, 0x00); }
	static const RGBColor Gray(uint8_t w)
	{
		double wd = 0xFF / 100.0 * w;
		int wi = int(0.5 + wd);
		if (wi > 0xFF) wi = 0xFF;
		return RGBColor(wi, wi, wi);
	}
};

class SDL_class
{
	SDL_Window *Window;
	SDL_Renderer *Renderer;

public:
	SDL_class(): Window(NULL), Renderer(NULL)
	{
		if (SDL_Init(SDL_INIT_VIDEO))
			Error("Could not initialize SDL");

		if (SDL_CreateWindowAndRenderer(0, 0, SDL_WINDOW_FULLSCREEN_DESKTOP,
										&Window, &Renderer))
			Error("Could not create window and renderer");

		if (SDL_RenderSetLogicalSize(Renderer, WINDOW_WIDTH, WINDOW_HEIGHT))
			Error("Could not set logical size");

		ClearScreen();
	};

	~SDL_class()
	{
		if (Renderer)
		{
			SDL_DestroyRenderer(Renderer);
			Renderer = NULL;
		}

		if (Window)
		{
			SDL_DestroyWindow(Window);
			Window = NULL;
		}

		SDL_Quit();
	};

	void SetPixel(int x, int y, const RGBColor &color = RGBColor::White())
	{
		if (SDL_SetRenderDrawColor(Renderer, color.r, color.g, color.b, 0xFF))
			Error("Could not set color");

		if (SDL_RenderDrawPoint(Renderer, x, y))
			Error("Could not draw point");
	};

	void ClearScreen()
	{
		if (SDL_SetRenderDrawColor(Renderer, 0, 0, 0, 0xFF))
			Error("Could not set color");

		if (SDL_RenderClear(Renderer))
			Error("Could not clear render");
	};

	void UpdateScreen() { SDL_RenderPresent(Renderer); }

	void Error(const char *msg)
	{
		cerr << "Error: " << msg << ": " << SDL_GetError() << endl;
		exit(1);
	};
};

typedef pair<Point, Point> PointPair;
typedef pair<Vector4, Vector4> V4Pair;


static SDL_class SDL;

static struct
{
	const Vector3 center = Vector3(10, 10, 50);
	const Vector3 top = Vector3(-10, -10, 200);
	const int R = 100;

	Matrix m_trans;

	const V4Pair lines[6] = {
		{{ -100, -10, 110 }, {   0,   0,  80 }},	// 1 сторона
		{{ -100,   5, 110 }, { 100,   5,  90 }},	// 2 стороны
		{{    5,  15,  10 }, {  20,  25,  80 }},	// основание
		{{ -100,  20, 110 }, { -10, -10, -10 }},	// сторона и основание
		{{  -20, -20,  80 }, {  20,  20,  90 }},	// внутри
		{{   80,  80,  10 }, {  90,  90,  50 }}		// снаружи
	};

	V4Pair res[6];
} test14_data;

// Переменные для анимации
static Matrix anim_rot, anim_proj;


// Helper function for Bezier_curve
int Dist(const Point p)
{
	return abs(p.x) + abs(p.y);
}

// Linear interpolation of two values
double Mix(const double start, const double end, const double t)
{
	return start + (end - start) * t;
}

unsigned long long Factorial(const int n)
{
	unsigned long long res = 1;

	for (int i = 2; i <= n; i++)
		res *= i;

	return res;
}

// Вычисление многочлена Бернштейна
double Bernstein(const int m, const int i, const double t)
{
	return Factorial(m) / Factorial(i) / Factorial(m - i)
		   * pow(t, i) * pow(1 - t, m - i);
}

// Определение положения точки p относительно отрезка прямой p1-p2
ClPointType Classify(const Point p1, const Point p2, const Point p)
{
	if (p == p1) return ORIGN;
	if (p == p2) return DEST;

	int ax = p2.x - p1.x,
		ay = p2.y - p1.y,
		bx = p.x - p1.x,
		by = p.y - p1.y,
		s = ax * by - bx * ay;	// Псевдоскалярное произведение

	if (0 > s) return LEFT;		// Слева от прямой
	if (0 < s) return RIGHT;	// Справа от прямой

	// Если s == 0, то p лежит на прямой.

	if (0 > ax * bx || 0 > ay * by)
		// Векторы p1-p2 и p1-p противонаправлены => p лежит за p1
		return BEHIND;

	if (ax * ax + ay * ay < bx * bx + by * by)
		// Вектор p1-p длиннее p1-p2 => p лежит за p2
		return BEYOND;

	return BETWEEN;				// p лежит между p1 и p2
}

// Определение точки пересечения прямых a-b и c-d
IntersectType Intersect(const Point a, const Point b,
						const Point c, const Point d, double &t)
{
	int nx = d.y - c.y,
		ny = c.x - d.x;

	int denom = nx * (b.x - a.x) + ny * (b.y - a.y);

	if (0 == denom)
	{
		ClPointType type = Classify(c, d, a);
		if (LEFT == type || RIGHT == type )
			return PARALLEL;
		else
			return COLLINEAR;
	}

	double num = nx * (a.x - c.x) + ny * (a.y - c.y);
	t = - num / denom;

	return SKEW;
}

// Определение факта пересечения отрезков a-b и c-d
IntersectType Cross(const Point a, const Point b, const Point c, const Point d)
{
	double tab = 0.0;
	IntersectType type = Intersect(a, b, c, d, tab);

	if (COLLINEAR == type || PARALLEL == type)
		return type;

	if (0.0 > tab || 1.0 < tab)
		return SKEW_NO_CROSS;

	double tcd = 0.0;
	Intersect(c, d, b, a, tcd);

	if (0.0 > tcd || 1.0 < tcd)
		return SKEW_NO_CROSS;

	return SKEW_CROSS;
}

// Классификация рёбер полигона
EType EdgeType(const Point p0, const Point p1, const Point a)
{
	switch (Classify(p0, p1, a))
	{
		case LEFT:
			if (a.y > p0.y && a.y <= p1.y)
				return CROSS_LEFT;
			else
				return INESSENTIAL;
		case RIGHT:
			if (a.y > p1.y && a.y <= p0.y)
				return CROSS_RIGHT;
			else
				return INESSENTIAL;
		case BETWEEN:
		case ORIGN:
		case DEST:
			return TOUCHING;
		default:
			return INESSENTIAL;
	}
}

// Вычерчивание отрезка прямой линии толщиной в 1 пиксел
// (алгоритм Брезенхэма)
void Line_1px(const Point a, const Point b,
			  const RGBColor &color = RGBColor::White())
{
	int x0 = a.x, y0 = a.y,
		x1 = b.x, y1 = b.y;

	bool xy = true;

	if (abs(y1 - y0) > abs(x1 - x0))
	{
		swap(x0, y0);
		swap(x1, y1);
		xy = false;
	}

	if (y0 > y1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}

	int d = 0,
		dx = abs(x1 - x0),
		dy = abs(y1 - y0),
		sx = (x0 > x1) ? -1 : 1,
		x = x0,
		y = y0;

	if (xy) SDL.SetPixel(x, y, color);
	else SDL.SetPixel(y, x, color);

	while (x != x1)
	{
		x += sx;
		d += 2 * dy;
		if (d > dx)
		{
			y++;
			d -= 2 * dx;
		}
		if (xy) SDL.SetPixel(x, y, color);
		else SDL.SetPixel(y, x, color);
	}
}

// Отрисовка полигона
void Polygon_stroke(const Polygon &p, const RGBColor &color = RGBColor::White())
{
	for (int i = 0; p.count() > i; i++)
		Line_1px(p[i], p[(1 + i) % p.count()], color);
}

// Определение типа полигона (простой / сложный)
PolygonType1 Polygon_type_1(const Polygon &p)
{
	for (int i = 0; p.count() > i; i++)
		for (int j = i; p.count() > j; j++)
			if (j != i && j != (1 + i) % p.count()
				&& (0 != i || j != p.count() - 1))
				if (SKEW_CROSS == Cross(p[i], p[(1 + i) % p.count()],
										p[j], p[(1 + j) % p.count()]))
					return COMPLEX;

	return SIMPLE;
}

// Определение типа полигона (выпуклый / невыпуклый)
PolygonType2 Polygon_type_2(const Polygon &p)
{
	for (int i = 0; p.count() > i; i++)
	{
		int left = 0,
			right = 0;

		for (int j = 0; p.count() > j; j++)
			if (j != i && j != (1 + i) % p.count())
			{
				if (LEFT == Classify(p[i], p[(1 + i) % p.count()], p[j]))
					left++;
				else
					right++;
			}

		if (0 < left && 0 < right) return CONCAVE;
	}

	return CONVEX;
}

void Print_polygon_type(const Polygon &p)
{
	cout << (SIMPLE == Polygon_type_1(p) ? "Простой " : "Сложный ")
		 << (CONVEX == Polygon_type_2(p) ? "выпуклый" : "невыпуклый") << endl;
}

// Определение габаритного прямоугольника
PointPair overall_rectangle(const Polygon &p)
{
	PointPair rect(p[0], p[0]);

	for (int i = 1; p.count() > i; i++)
	{
		if (rect.first.x > p[i].x)
			rect.first.x = p[i].x;

		if (rect.first.y > p[i].y)
			rect.first.y = p[i].y;

		if (rect.second.x < p[i].x)
			rect.second.x = p[i].x;

		if (rect.second.y < p[i].y)
			rect.second.y = p[i].y;
	}

	return rect;
}

// Правило even-odd
PType even_odd(const Polygon &p, const Point a)
{
	int winding_number = 0;
	for (int i = 0; p.count() > i; i++)
		switch (EdgeType(p[i], p[(1 + i) % p.count()], a))
		{
			case TOUCHING:
				return INSIDE;
			case CROSS_LEFT:
			case CROSS_RIGHT:
				winding_number = 1 - winding_number;
			case INESSENTIAL:
				break;
		}

	return winding_number ? INSIDE : OUTSIDE;
}

// Правило non-zero winding
PType non_zero_winding(const Polygon &p, const Point a)
{
	int winding_number = 0;
	for (int i = 0; p.count() > i; i++)
		switch (EdgeType(p[i], p[(1 + i) % p.count()], a))
		{
			case CROSS_LEFT:
				winding_number++;
				break;
			case CROSS_RIGHT:
				winding_number--;
				break;
			case TOUCHING:
			case INESSENTIAL:
				break;
		}

	return winding_number ? INSIDE : OUTSIDE;
}

// Заполнение полигона
void Polygon_fill(const Polygon &p, PType (*f)(const Polygon &, const Point),
				  const RGBColor &color = RGBColor::White())
{
	PointPair rect = overall_rectangle(p);

	for (int x = rect.first.x; rect.second.x >= x; x++)
		for (int y = rect.first.y; rect.second.y >= y; y++)
			if (INSIDE == (*f)(p, { x, y }))
				SDL.SetPixel(x, y, color);
}

// Вычерчивание отрезка прямой линии произвольной толщины
void Line(const Point a, const Point b, const RGBColor &color = RGBColor::White(),
		  const int w = 1, const LineCapsType line_caps_type = FLAT)
{
	if (1 == w)
		return Line_1px(a, b, color);

	int x0 = a.x, y0 = a.y,
		x1 = b.x, y1 = b.y;

	double m = w / 2;

	int	dx = x1 - x0,
		dy = y1 - y0;

	double len = sqrt(dx * dx + dy * dy);

	int hx = int(0.5 + m * dy / len),
		hy = int(0.5 + m * dx / len);

	if (SQUARE == line_caps_type)
	{
		int hx = int(0.5 + m * dx / len),
			hy = int(0.5 + m * dy / len);

		x0 -= hx;
		y0 -= hy;
		x1 += hx;
		y1 += hy;
	}

	if (ROUND == line_caps_type)
	{
		const int n = 16;
		double a = M_PI / n;

		int x = x0 - hx,
			y = y0 + hy;

		Polygon p(2 * n - 1);

		for (int i = 0; i < n - 1; i++)
		{
			int xn = int(0.5 + x0 + (x - x0) * cos(a) + (y0 - y) * sin(a)),
				yn = int(0.5 + y0 + (y - y0) * cos(a) + (x - x0) * sin(a));

			p[i] = { xn, yn };

			x = xn;
			y = yn;
		}

		x = x1 + hx;
		y = y1 - hy;

		for (int i = n - 1; i < 2 * n - 1; i++)
		{
			int xn = int(0.5 + x1 + (x - x1) * cos(a) + (y1 - y) * sin(a)),
				yn = int(0.5 + y1 + (y - y1) * cos(a) + (x - x1) * sin(a));

			p[i] = { xn, yn };

			x = xn;
			y = yn;
		}

		Polygon_fill(p, even_odd, color);
	}
	else // FLAT or SQUARE
	{
		Polygon p(4);
		p[0] = { x0 + hx, y0 - hy };
		p[1] = { x0 - hx, y0 + hy };
		p[2] = { x1 - hx, y1 + hy };
		p[3] = { x1 + hx, y1 - hy };
		Polygon_fill(p, even_odd, color);
	}

	Line(a, b, RGBColor::Red());
}

// Вычерчивание отрезка прямой линии штрихами
void Line(const Point a, const Point b, const Hatch &h,
		  const RGBColor &color = RGBColor::White(),
		  const int w = 1, const LineCapsType line_caps_type = FLAT)
{
	int dx = b.x - a.x,
		dy = b.y - a.y;

	double len = sqrt(dx * dx + dy * dy),
		   hx = dx / len,
		   hy = dy / len;

	int sum = 0;
	for (int i = 0; i < h.count; i++)
		sum += h.h[i];

	int lines = int(len / sum);

	int x = a.x,
		y = a.y;

	for (int i = 0; i < lines; i++)
		for (int j = 0; j < h.count; j += 2)
		{
			double l_dx = 0.5 + h.h[j] * hx,
				   l_dy = 0.5 + h.h[j] * hy,
				   s_dx = h.h[1 + j] * hx,
				   s_dy = h.h[1 + j] * hy;

			Point p1 = { x, y };
			Point p2 = { int(x + l_dx), int(y + l_dy) };
			Line(p1, p2, color, w, line_caps_type);

			x += int(l_dx + s_dx);
			y += int(l_dy + s_dy);
		}
}

// Отсечение отрезка прямой выпуклым многоугольником
// (алгоритм Кируса-Бека)
void Clip_line(const Point &a, const Point &b, const Polygon &p)
{
	double t0 = 0.0, t1 = 1.0;

	for (int i = 0;	p.count() > i; i++)
	{
		int nx = p[i].y - p[(1 + i) % p.count()].y,
			ny = p[(1 + i) % p.count()].x - p[i].x;

		int denom = nx * (b.x - a.x) + ny * (b.y - a.y);

		if (0 != denom)
		{
			double num = nx * (a.x - p[i].x) + ny * (a.y - p[i].y);
			double t = - num / denom;

			if (0 < denom)
			{
				// Потенциально входящая точка
				if (t > t0) t0 = t;
			}
			else
				// Потенциально покидающая точка
				if (t < t1) t1 = t;
		}
		else
			// Отрезок a-b || грани многоугольника
			if (LEFT == Classify(p[i], p[(1 + i) % p.count()], a))
				// Отрезок a-b вне многоугольника
				return;
	}

	if (t1 < t0)
		return;

	int x0 = 0.5 + Mix(a.x, b.x, t0),
		y0 = 0.5 + Mix(a.y, b.y, t0),
		x1 = 0.5 + Mix(a.x, b.x, t1),
		y1 = 0.5 + Mix(a.y, b.y, t1);

	Line({ x0, y0 }, { x1, y1 }, RGBColor::Red());
}

// Отсечение простого многоугольника выпуклым многоугольником
// (алгоритм Сазерлэнда-Ходжмана)
void Clip_polygon(const Polygon &p, const Polygon &clipper, Polygon &new_p)
{
	Polygon old_p = p;

	for (int i = 0; clipper.count() > i; i++)
	{
		new_p = 0;
		Point e0 = clipper[i];
		Point e1 = clipper[(1 + i) % clipper.count()];

		for (int j = 0; old_p.count() > j; j++)
		{
			Point a = old_p[j];
			Point b = old_p[(1 + j) % old_p.count()];

			double t = 0.0;
			IntersectType intersect = Intersect(a, b, e0, e1, t);

			if ((SKEW == intersect) && (0.0 <= t) && (1.0 >= t))
				new_p += { int(0.5 + Mix(a.x, b.x, t)),
						   int(0.5 + Mix(a.y, b.y, t)) };

            if (RIGHT == Classify(e0, e1, b))
				new_p += b;
		}

		old_p = new_p;
	}
}

// Построение кривой Безье N-го порядка
void Bezier_curve(const Point b[], const int n = 3)
{
	for (int i = 0; i < n; i++)
		Line(b[i], b[1 + i], RGBColor::Red());

	double step;

	if (3 == n)
	{
		Point d0 = { b[0].x - 2 * b[1].x + b[2].x,
					 b[0].y - 2 * b[1].y + b[2].y };
		Point d1 = { b[1].x - 2 * b[2].x + b[3].x,
					 b[1].y - 2 * b[2].y + b[3].y };
		double d = max(Dist(d0), Dist(d1));
		step = 1 / (1 + sqrt(3 * d));
	}
	else
		step = 0.01;

	Point R_prev = b[0];

	for (double t = step; t < 1.0; t += step)
	{
		double Rx = 0.0, Ry = 0.0;

		for (int i = 0; i <= n; i++)
		{
			double B = Bernstein(n, i, t);
			Rx += B * b[i].x;
			Ry += B * b[i].y;
		}

		Point R = { int(0.5 + Rx), int(0.5 + Ry) };

		Line(R_prev, R);

		R_prev = R;
	}

	Line(R_prev, b[n]);
}

// Вычисление 3D координат точки на кривой Безье 3-го порядка
void Bezier_coord(const Vector4 b[], const double t, Vector4 &res)
{
	res = Vector4(0, 0, 0, 1);

	for (int i = 0; i <= 3; i++)
	{
		double B = Bernstein(3, i, t);

		res.x += B * b[i].x;
		res.y += B * b[i].y;
		res.z += B * b[i].z;
	}
}

// Построение кубической B-сплайновой кривой
void B_spline(const Point p0, const Point p1, const Point p2, const Point p3)
{
	Line(p0, p1, RGBColor::Red());
	Line(p1, p2, RGBColor::Red());
	Line(p2, p3, RGBColor::Red());

	const double step = 0.03;

	Point R_prev;
	R_prev.x = int(0.5 + p0.x / 6.0 + 4.0 / 6 * p1.x + p2.x / 6.0);
	R_prev.y = int(0.5 + p0.y / 6.0 + 4.0 / 6 * p1.y + p2.y / 6.0);

	for (double t = step; t <= 1.0; t += step)
	{
		double k0 = (1 - t) * (1 - t) * (1 - t) / 6,
			   k1 = (3 * t * t * t - 6 * t * t + 4) / 6,
			   k2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6,
			   k3 = t * t * t / 6;

		double Rx = k0 * p0.x + k1 * p1.x + k2 * p2.x + k3 * p3.x,
			   Ry = k0 * p0.y + k1 * p1.y + k2 * p2.y + k3 * p3.y;

		Point R = { int(0.5 + Rx), int(0.5 + Ry) };

		Line(R_prev, R);

		R_prev = R;
	}
}

// Матрица перемещения
void MovementMatrix(const Vector3 &v, Matrix &m)
{
	m = Matrix::Diagonal();

	m(3,0) = v.x;
	m(3,1) = v.y;
	m(3,2) = v.z;
}

// Матрица поворота вокруг оси axis на угол angle (в радианах)
void RotationMatrix(const Vector3 &axis, const double angle, Matrix &m)
{
	Vector3 norm_axis = axis;
	norm_axis.Normalize();

	double x = norm_axis.x,
		   y = norm_axis.y,
		   z = norm_axis.z,
		   c = cos(angle),
		   s = sin(angle);

	m = Matrix::Diagonal();

	m(0,0) = x * x + (1.0 - x * x) * c;
	m(0,1) = x * y * (1.0 - c) + z * s;
	m(0,2) = z * x * (1.0 - c) - y * s;

	m(1,0) = x * y * (1.0 - c) - z * s;
	m(1,1) = y * y + (1.0 - y * y) * c;
	m(1,2) = y * z * (1.0 - c) + x * s;

	m(2,0) = z * x * (1.0 - c) + y * s;
	m(2,1) = y * z * (1.0 - c) - x * s;
	m(2,2) = z * z + (1.0 - z * z) * c;
}

// Построение параллельной проекции повернутого параллелепипеда на плоскость Z=n
void ParallelProjectionMatrix(const double n, Matrix &m)
{
	m(0,0) = 1.0;
	m(1,1) = 1.0;
	m(3,2) = n;
	m(3,3) = 1.0;
}

// Построение одноточечной перспективной проекции повернутого параллелепипеда,
// центр проекции находится в точке [0, 0, k]
void PerspectiveProjectionMatrix(const double k, Matrix &m)
{
	m(0,0) = 1.0;
	m(1,1) = 1.0;
	m(2,2) = 1.0;
	m(2,3) = -1 / k;
	m(3,3) = 1.0;
}

void Line_3D(const Vector4 &start, const Vector4 &end, const Matrix &proj,
			 const RGBColor &color = RGBColor::White())
{
	Vector4 a = start * proj,
			b = end * proj;

	Point p1 = { int(0.5 + a.x), int(0.5 + a.y) },
		  p2 = { int(0.5 + b.x), int(0.5 + b.y) };

	Line(p1, p2, color);
}

void RPP_Draw(RPP &rpp, const Matrix &p, const bool transparent = true,
			  const RGBColor &color = RGBColor::White())
{
	Vector3 P(0, 0, -1 / p(2,3)),
			a1 = rpp.a1, b1 = rpp.b1, c1 = rpp.c1, d1 = rpp.d1,
			a2 = rpp.a2, b2 = rpp.b2, c2 = rpp.c2;

	// Нормали к граням
	Vector3 n_front = (a2 - a1) ^ (b2 - a1),
			n_right = (b2 - b1) ^ (c1 - b1),
			n_top   = (a2 - b2) ^ (c2 - b2);

	n_front.Normalize();
	n_right.Normalize();
	n_top.Normalize();

	Vector3 n_back = -n_front,
			n_left = -n_right,
			n_bottom = -n_top;

	// Front
	if (transparent || 0 > ((a1 + b2) / 2 - P) * n_front)
	{
		Line_3D(rpp.a1, rpp.b1, p, color);
		Line_3D(rpp.b1, rpp.b2, p, color);
		Line_3D(rpp.b2, rpp.a2, p, color);
		Line_3D(rpp.a2, rpp.a1, p, color);
	}

	// Right
	if (transparent || 0 > ((b1 + c2) / 2 - P) * n_right)
	{
		Line_3D(rpp.b1, rpp.b2, p, color);
		Line_3D(rpp.b2, rpp.c2, p, color);
		Line_3D(rpp.c2, rpp.c1, p, color);
		Line_3D(rpp.c1, rpp.b1, p, color);
	}

	// Top
	if (transparent || 0 > ((a2 + c2) / 2 - P) * n_top)
	{
		Line_3D(rpp.a2, rpp.b2, p, color);
		Line_3D(rpp.b2, rpp.c2, p, color);
		Line_3D(rpp.c2, rpp.d2, p, color);
		Line_3D(rpp.d2, rpp.a2, p, color);
	}

	// Back
	if (transparent || 0 > ((d1 + c2) / 2 - P) * n_back)
	{
		Line_3D(rpp.d1, rpp.c1, p, color);
		Line_3D(rpp.c1, rpp.c2, p, color);
		Line_3D(rpp.c2, rpp.d2, p, color);
		Line_3D(rpp.d2, rpp.d1, p, color);
	}

	// Left
	if (transparent || 0 > ((a2 + d1) / 2 - P) * n_left)
	{
		Line_3D(rpp.a1, rpp.a2, p, color);
		Line_3D(rpp.a2, rpp.d2, p, color);
		Line_3D(rpp.d2, rpp.d1, p, color);
		Line_3D(rpp.d1, rpp.a1, p, color);
	}

	// Bottom
	if (transparent || 0 > ((a1 + c1) / 2 - P) * n_bottom)
	{
		Line_3D(rpp.a1, rpp.b1, p, color);
		Line_3D(rpp.b1, rpp.c1, p, color);
		Line_3D(rpp.c1, rpp.d1, p, color);
		Line_3D(rpp.d1, rpp.a1, p, color);
	}
}

void Cone_draw(const double r, const double h, const Matrix &m,
			   const Matrix &proj, const RGBColor &color = RGBColor::White(),
			   const int segments = 16)
{
	Vector4 top(0, 0, 0, 1);
	Vector3 axis(0, 0, 1);
	double angle = 2.0 * M_PI / segments;
	Matrix rotate;

	for (int i = 0; segments > i; i++)
	{
		double angle_a = angle * i;
		double angle_b = angle * (i + 1);

		Vector4 a(0, r, h);
		RotationMatrix(axis, angle_a, rotate);
		a *= rotate;

		Vector4 b(0, r, h);
		RotationMatrix(axis, angle_b, rotate);
		b *= rotate;

		Vector4 new_top = top * m;
		Vector4 new_a = a * m;
		Vector4 new_b = b * m;

		Line_3D(new_top, new_b, proj, color);
		Line_3D(new_a, new_b, proj, color);
	}
}

// Отсечение отрезка прямой конусом
void Clip_cone(const Vector4 &line0, const Vector4 &line1, const double r,
			   const double h, Vector4 &res_p0, Vector4 &res_p1)
{
	double x1 = line0.x, y1 = line0.y, z1 = line0.z,
		   x2 = line1.x, y2 = line1.y, z2 = line1.z;

	double a = r / h,
		   b = r / h,
		   c = 1.0;

	double A, B, C;

	A = b*b*c*c*x2*x2 - 2*b*b*c*c*x1*x2 + b*b*c*c*x1*x1 +
		a*a*c*c*y2*y2 - 2*a*a*c*c*y1*y2 + a*a*c*c*y1*y1 -
		a*a*b*b*z2*z2 + 2*a*a*b*b*z1*z2 - a*a*b*b*z1*z1;

	B = 2*b*b*c*c*x1*x2 - 2*b*b*c*c*x1*x1 +
		2*a*a*c*c*y1*y2 - 2*a*a*c*c*y1*y1 -
		2*a*a*b*b*z1*z2 + 2*a*a*b*b*z1*z1;

	C = b*b*c*c*x1*x1 + a*a*c*c*y1*y1 - a*a*b*b*z1*z1;

	double t[3] = {0, 0, 0};
	int count = 0;

	double D = B*B - 4*A*C;

	if (0 <= D)
	{
		double t1 = (-B - sqrt(D)) / (2*A),
			   t2 = (-B + sqrt(D)) / (2*A);

		double z = Mix(z1, z2, t1);
		if (fabs(h) >= fabs(z)) t[count++] = ((0 <= t1) && (1 >= t1)) ? t1 : 0;

		z = Mix(z1, z2, t2);
		if (fabs(h) >= fabs(z)) t[count++] = ((0 <= t2) && (1 >= t2)) ? t2 : 1;
	}

	Vector3 n_p0(0, 0, h),
			n_p1(r, 0, h),
			n_p2(0, r, h);

	Vector3 vect = (n_p1 - n_p0) ^ (n_p2 - n_p0);

	Vector3 n = vect / vect.Length();

	Vector3 pl_c(r, r, h),
			pl_a = line0,
			pl_b = line1;

	double t_pl = (-n * (pl_a - pl_c)) / (n * (pl_b - pl_a));

	double x = Mix(x1, x2, t_pl),
		   y = Mix(y1, y2, t_pl);

	if ((r >= x) && (-r <= x) && (r >= y) && (-r <= y))
		t[count++] = t_pl;

	if (0 == count)
	{
		res_p0 = Vector4(0, 0, 0, 0);
		res_p1 = Vector4(0, 0, 0, 0);
		return;
	}

	if (z2 < z1) swap(t[0], t[1]);

	double p0_x = Mix(x1, x2, t[0]),
		   p0_y = Mix(y1, y2, t[0]),
		   p0_z = Mix(z1, z2, t[0]);
	res_p0 = Vector4(p0_x, p0_y, p0_z);

	double p1_x = Mix(x1, x2, t[count - 1]),
		   p1_y = Mix(y1, y2, t[count - 1]),
		   p1_z = Mix(z1, z2, t[count - 1]);
	res_p1 = Vector4(p1_x, p1_y, p1_z);
}


void Test1()
{
	Point p = { 250, 90 };
	const RGBColor c = RGBColor::Acid();
	Line(p, { p.x + 8, p.y + 3 }, c);	// 1 октант
	Line(p, { p.x + 3, p.y + 8 }, c);	// 2 октант
	Line(p, { p.x - 3, p.y + 8 }, c);	// 3 октант
	Line(p, { p.x - 8, p.y + 3 }, c);	// 4 октант
	Line(p, { p.x - 8, p.y - 3 }, c);	// 5 октант
	Line(p, { p.x - 3, p.y - 8 }, c);	// 6 октант
	Line(p, { p.x + 3, p.y - 8 }, c);	// 7 октант
	Line(p, { p.x + 8, p.y - 3 }, c);	// 8 октант

	Line({  10,  10 }, { 110,  60 });	// 1 октант
	Line({  30,  30 }, {  60, 110 });	// 2 октант
	Line({ 170,  20 }, { 120, 100 });	// 3 октант
	Line({ 250,  30 }, { 200,  50 });	// 4 октант
	Line({ 180, 150 }, {  80,  80 });	// 5 октант
	Line({ 250, 180 }, { 200, 100 });	// 6 октант
	Line({  40, 200 }, {  80, 130 });	// 7 октант
	Line({ 120, 200 }, { 200, 160 });	// 8 октант
	Line({ 100,  30 }, { 100,  30 });	// Точка
	Line({  80, 170 }, { 140, 170 });	// Горизонтальная
	Line({ 130, 180 }, {  70, 180 });	// Горизонтальная
	Line({ 175,  50 }, { 175, 110 });	// Вертикальная
	Line({  15, 160 }, {  15,  70 });	// Вертикальная

	Line({ 250, 200 }, { 300, 400 }, RGBColor::Gray(90), 90);
	Line({ 300,  90 }, { 680, 420 }, RGBColor::Gray(50), 50);
	Line({ 350,  30 }, { 700,  30 }, RGBColor::Gray(25), 25);
	Line({ 100, 220 }, { 100, 440 }, RGBColor::White(), 12);
}

void Test2()
{
	RGBColor c = RGBColor::White();
	Line({  50, 40 }, { 150, 400 }, c, 40, FLAT);
	Line({ 150, 40 }, { 250, 400 }, c, 40, ROUND);
	Line({ 250, 40 }, { 350, 400 }, c, 40, SQUARE);

	c = RGBColor::Gray(80);
	Line({ 400, 40 }, { 450, 400 }, c, 10, FLAT);
	Line({ 500, 40 }, { 550, 400 }, c, 10, ROUND);
	Line({ 600, 40 }, { 650, 400 }, c, 10, SQUARE);

	Hatch h1 = { 2, { 30, 30 } };
	Hatch h2 = { 4, { 30, 30, 50, 30 } };
	Hatch h3 = { 6, { 5, 10, 15, 20, 25, 30 } };
	Line({ 100,  50 }, { 700, 400 }, h1, RGBColor::Acid(),  20, SQUARE);
	Line({  20, 150 }, { 650, 420 }, h2, RGBColor::Green(), 20, ROUND);
	Line({ 320, 100 }, { 720, 180 }, h3, RGBColor::Blue(),  10, FLAT);
}

void Test3()
{
	Polygon p;
	p += { 100,  10 };
	p += { 300,  10 };
	p += { 200, 110 };
	p += { 200, 210 };
	Print_polygon_type(p);	// Простой, невыпуклый
	Polygon_stroke(p);

	p = 0;
	p += { 210, 120 };
	p += { 410,  20 };
	p += { 460, 300 };
	p += { 210, 220 };
	Print_polygon_type(p);	// Простой, выпуклый
	Polygon_fill(p, even_odd, RGBColor::Gray(50));

	p = 0;
	p += { 405, 295 };
	p += {  98, 380 };
	p += { 259, 442 };
	p += { 207, 354 };
	p += { 300, 194 };
	p += { 325, 194 };
	p += { 195, 445 };
	p += {   8, 440 };
	p += { 340, 196 };
	p += { 189, 428 };
	Print_polygon_type(p);	// Сложный, невыпуклый
	Polygon_fill(p, even_odd);
	Polygon_stroke(p, RGBColor::Gray(50));

	for (int i = 0; i < p.count(); i++)
		p[i].x += 300;
	Print_polygon_type(p);	// Сложный, невыпуклый
	Polygon_fill(p, non_zero_winding);
	Polygon_stroke(p, RGBColor::Gray(50));
}

void Test4()
{
	Polygon p(15);
	p[0] = { 170, 210 };
	p[1] = {  20,  20 };
	p[2] = { 320, 210 };
	p[3] = {  20, 400 };
	p[4] = { 320,  20 };
	p[5] = {  20, 210 };
	p[6] = { 320, 400 };
	p[7] = { 170, 210 };
	p[8] = {  20, 115 };
	p[9] = { 170,  20 };
	p[10] = { 320, 115 };
	p[11] = { 170, 210 };
	p[12] = {  20, 305 };
	p[13] = { 170, 400 };
	p[14] = { 320, 305 };
	Print_polygon_type(p);	// Сложный, невыпуклый
	Polygon_fill(p, even_odd);
	Polygon_stroke(p, RGBColor::Gray(80));

	for (int i = 0; i < p.count(); i++)
		p[i].x += 350;
	Print_polygon_type(p);	// Сложный, невыпуклый
	Polygon_fill(p, non_zero_winding);
	Polygon_stroke(p, RGBColor::Gray(80));
}

void Test5()
{
	Polygon p;
	p += { 200, 200 };
	p += { 400, 100 };
	p += { 500, 400 };
	p += { 200, 300 };
	Polygon_stroke(p);

	Point a, b;
	a = {  80, 200 };
	b = { 200, 100 };
	Line(a, b);
	Clip_line(a, b, p);

	a = { 300,  80 };
	b = { 300, 200 };
	Line(a, b);
	Clip_line(a, b, p);

	a = { 100, 300 };
	b = { 600, 150 };
	Line(a, b);
	Clip_line(a, b, p);

	a = { 350, 280 };
	b = { 450, 340 };
	Line(a, b);
	Clip_line(a, b, p);
}

void Test_clip_polygon(const Polygon &p0, const Polygon &p1)
{
	Polygon res;
	Clip_polygon(p0, p1, res);
	Polygon_stroke(p0,  RGBColor::Blue());
	Polygon_stroke(p1,  RGBColor::Red());
	Polygon_stroke(res, RGBColor::Magenta());
}

void Test6()
{
	Polygon p0;	// отсекаемый объект
	p0 += { 100, 250 };
	p0 += { 480,  30 };
	p0 += { 300, 250 };
	p0 += { 430, 430 };
	Polygon p1;	// отсекатель
	p1 += { 200, 400 };
	p1 += { 200, 100 };
	p1 += { 400, 100 };
	p1 += { 400, 400 };
	Test_clip_polygon(p0, p1);

	p0 = 0;
	p0 +=  { 40, 320 };
	p0 += { 80, 320 };
	p0 += { 60, 340 };
	p0 += { 60, 360 };
	p1 = 0;
	p1 += {  30, 310 };
	p1 += { 100, 320 };
	p1 += { 120, 350 };
	p1 += {  90, 380 };
	p1 += {  50, 400 };
	Test_clip_polygon(p0, p1);

	p0 = 0;
	p0 += {  30,  40 };
	p0 += {  54,  50 };
	p0 += { 120,  30 };
	p0 += { 110, 150 };
	p0 += {  34, 150 };
	p1 = 0;
	p1 += {  50,  70 };
	p1 += {  90,  45 };
	p1 += { 110,  80 };
	p1 += {  80, 120 };
	p1 += {  40, 140 };
	Test_clip_polygon(p0, p1);

	p0 = 0;
	p0 += { 620, 220 };
	p0 += { 640, 240 };
	p0 += { 630, 300 };
	p0 += { 600, 300 };
	p1 = 0;
	p1 += { 540, 240 };
	p1 += { 580, 220 };
	p1 += { 600, 250 };
	p1 += { 570, 280 };
	p1 += { 530, 290 };
	Test_clip_polygon(p0, p1);
}

void Test7()
{
	Point b[4];

	b[0] = { 100, 400 };
	b[1] = { 200, 250 };
	b[2] = { 350, 400 };
	Bezier_curve(b, 2);

	b[0] = {  20, 100 };
	b[1] = { 100, 120 };
	b[2] = {  90,  40 };
	b[3] = { 180,  60 };
	Bezier_curve(b);

	b[0] = { 150, 200 };
	b[1] = { 300, 100 };
	b[2] = { 320, 230 };
	b[3] = { 250, 180 };
	Bezier_curve(b);

	b[0] = { 380, 200 };
	b[1] = { 600, 250 };
	b[2] = { 650,  50 };
	b[3] = { 500, 180 };
	Bezier_curve(b);
}

void Test8()
{
	Point b[4];

	b[0] = {  50, 230 };
	b[1] = { 150,  80 };
	b[2] = { 350,  30 };
	b[3] = { 400, 230 };
	Bezier_curve(b);

	b[0] = { 400, 230 };
	b[1] = { 450, 430 };
	b[2] = { 550, 380 };
	b[3] = { 550, 130 };
	Bezier_curve(b);
}

void Test9()
{
	Point b[6];

	b[0] = {  50, 170 };
	b[1] = { 150,  20 };
	b[2] = { 200, 370 };
	b[3] = { 450,  20 };
	b[4] = { 600, 270 };
	Bezier_curve(b, 4);

	b[0] = {  50, 230 };
	b[1] = { 150,  80 };
	b[2] = { 200, 430 };
	b[3] = { 450,  80 };
	b[4] = { 600, 330 };
	b[5] = { 300, 430 };
	Bezier_curve(b, 5);
}

void Test10()
{
	Point p0, p1, p2, p3, p4, p5, p6;
	p0 = {  50, 250 };
	p1 = { 150, 100 };
	p2 = { 350,  50 };
	p3 = { 450, 250 };
	B_spline(p0, p1, p2, p3);

	p4 = { 300, 300 };
	B_spline(p1, p2, p3, p4);

	p5 = { 550, 350 };
	B_spline(p2, p3, p4, p5);

	p6 = { 600, 100 };
	B_spline(p3, p4, p5, p6);
}

void Test11_12(int test_num)
{
	RPP rpp(50, 100, 150);

	Matrix m_rot;
	Vector3 axis(0.5, 1, 1);
	RotationMatrix(axis, 1.0, m_rot);
	rpp *= m_rot;

	Matrix m_proj;
	if (11 == test_num)
		ParallelProjectionMatrix(100, m_proj);
	else
		PerspectiveProjectionMatrix(500, m_proj);

	Matrix m_move;
	Vector3 move(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0);
	MovementMatrix(move, m_move);
	m_proj *= m_move;

	RPP_Draw(rpp, m_proj, 11 == test_num);
}

void Test13()
{
	Vector3 axis(1, 1, 1);
	RotationMatrix(axis, 0.0, anim_rot);

	PerspectiveProjectionMatrix(500, anim_proj);

	Matrix m_move;
	Vector3 move(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0);
	MovementMatrix(move, m_move);
	anim_proj *= m_move;
}

void Test14()
{
	Test13();

	const Vector3 &center = test14_data.center;
	const Vector3 &top = test14_data.top;
	const double h = -(top - center).Length();
	Matrix &m_trans = test14_data.m_trans;

	Vector3 a1(0, 0, 1);
	Vector3 a2(top.x - center.x, top.y - center.y, top.z - center.z);
	double phi = acos(a1 * a2 / a1.Length() / a2.Length());
	Vector3 mv(top.x, top.y, top.z);
	MovementMatrix(mv, m_trans);

	Matrix m_rot;
	Vector3 axis = Vector3(0, 0, 1);
	RotationMatrix(axis, phi, m_rot);
	m_trans *= m_rot;

	Matrix m_trans_inv;
	MovementMatrix(-mv, m_trans_inv);
	axis = Vector3(0, 0, 1);
	RotationMatrix(axis, -phi, m_rot);
	m_trans_inv *= m_rot;

	for (int i = 0; i < 6; i++)
	{
		const V4Pair &line = test14_data.lines[i];
		Vector4 p0 = line.first  * m_trans_inv, res_p0,
				p1 = line.second * m_trans_inv, res_p1;
		Clip_cone(p0, p1, test14_data.R, h, res_p0, res_p1);
		V4Pair &res = test14_data.res[i];
		res.first  = res_p0 * m_trans;
		res.second = res_p1 * m_trans;
	}
}

void InitScene(int scene_num)
{
	switch (scene_num)
	{
		case 1:  return Test1();
		case 2:  return Test2();
		case 3:  return Test3();
		case 4:  return Test4();
		case 5:  return Test5();
		case 6:  return Test6();
		case 7:  return Test7();
		case 8:  return Test8();
		case 9:  return Test9();
		case 10: return Test10();
		case 11:
		case 12: return Test11_12(scene_num);
		case 13: return Test13();
		case 14: return Test14();
	}
}

void DrawScene13()
{
	static double t = 0, step = 0.02;

	Vector4 b[4];
	b[0] = Vector4(-250, -50, -200);
	b[1] = Vector4(-100, -200, 50);
	b[2] = Vector4(50, -150, 100);
	b[3] = Vector4(200, 50, -300);

	Vector4 coord;
	Bezier_coord(b, t, coord);

	Matrix m_rot;
	Vector3 axis(coord.x, coord.y, coord.z);
	RotationMatrix(axis, 0.1, m_rot);
	anim_rot *= m_rot;

	Matrix m_move;
	Vector3 move(coord.x, coord.y, coord.z);
	MovementMatrix(move, m_move);
	Matrix m_trans = anim_rot * m_move;

	Matrix m_proj;
	PerspectiveProjectionMatrix(500, m_proj);
	move = Vector3(400, 300, 0);
	MovementMatrix(move, m_move);
	m_proj *= m_move;

	Vector4 start = b[0], end;
	for (double i = 0.01; i < 1; i += 0.01)
	{
		Bezier_coord(b, i, end);
		Line_3D(start, end, m_proj, RGBColor::Gray(50));
		start = end;
	}

	Vector4 null(0, 0, 0, 1);
	Line_3D(null, coord, m_proj, RGBColor::Green());

	RPP rpp(50, 100, 150);
	rpp *= m_trans;
	RPP_Draw(rpp, m_proj, false);

	t += step;
	if ((t > 1) || (t < 0)) step = -step;
}

void DrawScene14()
{
	const Vector3 &center = test14_data.center;
	const Vector3 &top = test14_data.top;
	const double h = -(top - center).Length();
	const Matrix &m_trans = test14_data.m_trans;

	Matrix m_rot;
	Vector3 axis(0, 1, 1);
	RotationMatrix(axis, 0.05, m_rot);
	anim_rot *= m_rot;

	Matrix m_proj = anim_rot * anim_proj;

	for (int i = 0; i < 6; i++)
	{
		const V4Pair &line = test14_data.lines[i];
		Line_3D(line.first, line.second, m_proj);
		const V4Pair &res = test14_data.res[i];
		Line_3D(res.first, res.second, m_proj, RGBColor::Red());
	}

	Cone_draw(test14_data.R, h, m_trans, m_proj, RGBColor::Gray(50));
}

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cerr << "Usage: " << argv[0] << " [test number (1-14)]" << endl;
		return 1;
	}

	int test_num = atoi(argv[1]);
	if (test_num < 1 || test_num > 14)
	{
		cerr << "Wrong test number (" << test_num << ")" << endl;
		return 1;
	}

	InitScene(test_num);
	SDL.UpdateScreen();

	bool run = true;
	while (run)
	{
		if (test_num >= 13)
		{
			SDL.ClearScreen();
			if (test_num == 13)
				DrawScene13();
			else
				DrawScene14();
			SDL.UpdateScreen();
		}

		SDL_Event event;
		while (SDL_PollEvent(&event))
			if (event.type == SDL_QUIT)
				run = false;

		SDL_Delay(100);
	}

	return 0;
}
