#pragma once
#include <DirectXMath.h>

DirectX::XMFLOAT3 operator*(DirectX::XMFLOAT3 a, DirectX::XMFLOAT3 b) 
{
	return 	{ a.x * b.x,		a.y * b.y,		a.z * b.z };
}

DirectX::XMFLOAT3 operator+(DirectX::XMFLOAT3 a, DirectX::XMFLOAT3 b)
{
	return 	{	a.x + b.x,		a.y + b.y,		a.z + b.z	};
}
DirectX::XMFLOAT3 operator-(DirectX::XMFLOAT3 a, DirectX::XMFLOAT3 b)
{
	return 	{	a.x - b.x,		a.y - b.y,		a.z - b.z	};
}

DirectX::XMFLOAT3 Lerp(const DirectX::XMVECTOR& a, DirectX::XMFLOAT3 b, float t)
{
	DirectX::XMVECTOR v = DirectX::XMVectorLerp(a,DirectX::XMLoadFloat3(&b),t);
	DirectX::XMStoreFloat3(&b, v);

	return b;

}
