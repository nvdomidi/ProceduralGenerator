#pragma once

#include <limits>

class BoundingBox2D {
	FVector2D minCorner, maxCorner;

public:

	BoundingBox2D(float minX, float maxX, float minY, float maxY) {
		this->minCorner = FVector2D{ minX, minY };
		this->maxCorner = FVector2D{ maxX, maxY };
	}

	FVector2D getSize() {
		return FVector2D{ FMath::Abs(this->maxCorner[0] - this->minCorner[0]), FMath::Abs(this->maxCorner[1] - this->minCorner[1]) };
	}

	float getArea() {
		return FMath::Abs(this->maxCorner[0] - this->minCorner[0]) * FMath::Abs(this->maxCorner[1] - this->minCorner[1]);
	}

	FVector2D getCenter() {
		float x = this->minCorner[0] + FMath::Abs(this->maxCorner[0] - this->minCorner[0]) * 0.5;
		float y = this->minCorner[1] + FMath::Abs(this->maxCorner[1] - this->minCorner[1]) * 0.5;

		return FVector2D{ x,y };
	}

	void fromFVectorList(TArray<FVector> face) {
		float minX = std::numeric_limits<float>::max();
		float maxX = -std::numeric_limits<float>::max();
		float minY = std::numeric_limits<float>::max();
		float maxY = -std::numeric_limits<float>::max();

		for (int i = 0; i < face.Num(); i++) {
			minX = FMath::Min(face[i].X, minX);
			maxX = FMath::Max(face[i].X, maxX);
			minY = FMath::Min(face[i].Y, minY);
			maxY = FMath::Max(face[i].Y, maxY);
		}

		this->minCorner = FVector2D{ minX, minY };
		this->maxCorner = FVector2D{ maxX, maxY };
	}

};

class OrientedBoundingBox2D {
	
public:
	FVector2D pos; // center point
	float rot_angle;
	FVector2D extents;
	FVector2D long_axis;
	FVector2D short_axis;
	TArray<FVector2D> corners;

public:
	OrientedBoundingBox2D() {
		this->pos = FVector2D{};
		this->rot_angle = 0.0f;
		this->extents = FVector2D{};
		this->long_axis = FVector2D{};
		this->short_axis = FVector2D{};
		this->corners = TArray<FVector2D>{};
	}

	float getArea() {
		return this->extents.X * this->extents.Y;
	}

	// this function transforms corners on unit box to OBB box
	// e.g: input: {-0.5,-0.5} --> output: bottom left corner on OBB
	FVector2D transformPoint(FVector2D p) {
		float deltaX = this->pos.X;
		float deltaY = this->pos.Y;
		float theta = this->rot_angle; // TODO: check -this->rot_angle; 
		float Sx = this->extents.X;
		float Sy = this->extents.Y;

		float xx = Sx * FMath::Cos(theta) * p.X - Sy * FMath::Sin(theta) * p.Y + deltaX;
		float yy = Sx * FMath::Sin(theta) * p.X + Sy * FMath::Cos(theta) * p.Y + deltaY;

		return FVector2D{ xx,yy };
	}

	void getCorners() {
		this->corners = TArray<FVector2D>{};

		// clockwise from bottom left
		FVector2D p1{ -0.5f, -0.5f };
		FVector2D p2{ -0.5f, 0.5f };
		FVector2D p3{ 0.5f, 0.5f };
		FVector2D p4{ 0.5f, -0.5f };
	
		p1 = transformPoint(p1);
		p2 = transformPoint(p2);
		p3 = transformPoint(p3);
		p4 = transformPoint(p4);

		this->corners.Add(p1);
		this->corners.Add(p2);
		this->corners.Add(p3);
		this->corners.Add(p4);

	}

	void getAxes() {
		FVector2D p1 = this->corners[0];
		FVector2D p2 = this->corners[1];
		FVector2D p3 = this->corners[2];

		float p1p2 = FVector2D::Distance(p1, p2);
		float p2p3 = FVector2D::Distance(p2, p3);

		FVector2D dir{};
		FVector2D ortho{};

		if (p1p2 < p2p3) {
			dir = p2 - p1;
			ortho = p3 - p2;
		}
		else {
			dir = p3 - p2;
			ortho = p2 - p1;
		}

		dir.Normalize();
		ortho.Normalize();

		this->short_axis = dir;
		this->long_axis = ortho;
	}

	// takes a face as input e.g: {1,10,20,30,1} and finds its obb
	void getMinimumFromFace(TArray<FVector> face) {
		float minArea = std::numeric_limits<float>::max();
		BoundingBox2D bb(0, 0, 0, 0);
		bb.fromFVectorList(face);
		FVector2D center = bb.getCenter();
		this->pos = center;
		for (int i = 0; i < face.Num(); i++) {
			FVector2D p1{ face[i].X, face[i].Y };
			FVector2D p2{ face[(i+1)%face.Num()].X, face[(i + 1) % face.Num()].Y};
			FVector2D e1 = p2 - p1;

			UE_LOG(LogTemp, Warning, TEXT("p1: {%f, %f}, p2: {%f, %f}"), p1.X, p1.Y, p2.X, p2.Y);

			e1.Normalize();

			FVector2D dir = e1;

			FVector2D xAxis{ 1,0 };

			
			int o = FMath::Sign(-dir.Y); // TODO: check  FMath::Sign(-dir.Y);
			float d = FVector2D::DotProduct(dir, xAxis);

			float angle = FMath::Acos(d);

			if (o == -1)
				angle *= -1;

			// rotate the face to be in the space of edge
			TArray<FVector> rotated{};

			for (int j = 0; j < face.Num(); j++) {
				FVector c1{ face[j].X, face[j].Y, 0.0f };
				// rotate
				float c1x = c1.X - center.X;
				float c1y = c1.Y - center.Y;
				float sinC = FMath::Sin(angle);
				float cosC = FMath::Cos(angle);

				c1.X = c1x * cosC - c1y * sinC + center.X;
				c1.Y = c1x * sinC + c1y * cosC + center.Y;
				// rotated
				rotated.Add(c1);
			}

			BoundingBox2D curBB(0, 0, 0, 0);
			curBB.fromFVectorList(rotated);
			float currArea = curBB.getArea();
			if (currArea < minArea) {
				this->extents = curBB.getSize();
				this->rot_angle = -angle;
				minArea = currArea;
			}

		}

		this->getCorners();
		this->getAxes();
	}
	
};
