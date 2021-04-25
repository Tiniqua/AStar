#pragma once
#include <SpriteFont.h>
#include <array>

#ifndef FONT_SIZE
#define FONT_SIZE 2
#endif // !FONT_SIZE
#include "DescriptorHeap.h"

#include <wrl.h>
#include "TextConsole.h"
#include "Common/d3dx12.h"
#include <GraphicsMemory.h>
#include <d3d12.h>
#include <CommonStates.h>
#include "SimpleMath.h"

struct Text
{
	std::string string = "";
	DirectX::SimpleMath::Vector2 position = { 0,0 };
	DirectX::SimpleMath::Vector4 color = { 0.0f,0.0f,0.0f,1.0f };
	float rotation = 0.0f;
	DirectX::SimpleMath::Vector2 origin = { 0,0 };
	float scale = 1.0f;
	size_t fontIndex = 0;
	bool center = false;

	void operator=(const Text& text)
	{
		string = text.string;
		position = text.position;
		color = text.color;
		rotation = text.rotation;
		origin = text.origin;
		scale = text.scale;
		fontIndex = text.fontIndex;
		center = text.center;
	}

	void Draw();
};

struct Sprite
{
	std::string textureName; //lookup gpu handle
	D3D12_GPU_DESCRIPTOR_HANDLE texture;
	DirectX::XMUINT2 textureSize = DirectX::XMUINT2(512, 512);
	DirectX::XMFLOAT2 position = { 0.0f,0.0f };
	RECT destinationRectangle = { -1,-1,-1,-1 }; //ignore if left = -1
	RECT sourceRectangle = { 0,0,512,512 };   
	DirectX::SimpleMath::Vector4 color = DirectX::Colors::White;
	float rotation = 0.0f;
	float scale = 1.0f;
	DirectX::XMFLOAT2 origin = DirectX::XMFLOAT2(0.0f, 0.0f);
	DirectX::SpriteEffects effects = DirectX::SpriteEffects::SpriteEffects_None;

	Sprite() {};
	Sprite(const std::string& textureName);

	void operator=(const Sprite& s)
	{
		textureName = s.textureName;
		textureSize = s.textureSize;
		texture = s.texture;
		position = s.position;
		destinationRectangle = s.destinationRectangle;
		sourceRectangle = s.sourceRectangle;
		rotation = s.rotation;
		scale = s.scale;
		origin = s.origin;
		effects = s.effects;
		//color = s.color;
	}

	// Give a valid texture name
	void Initialise(const std::string& textureName, bool centreOrigin = false);

	void Draw();


};

class Panel
{
	// array names
	enum
	{
		CORNER_TL, CORNER_TR, CORNER_BL, CORNER_BR,
		EDGE_L, EDGE_T, EDGE_R, EDGE_B,
		MIDDLE, COUNT
	};

	// sprites making up the panel display
	std::array<Sprite, COUNT> mSprites;
	RECT mSourceRect = { 0,0,512,512 };
	// Screen scace of panel
	RECT mDestRect = { 0,0,512,512 };


	void CalcSpriteRects();


public:
	void Initialize(const std::string& textureName, const RECT& src, const RECT& dst);

	void Move(DirectX::SimpleMath::Vector2 v, bool increment = true);
	const DirectX::SimpleMath::Vector2 GetPos() const;
	void Draw();
};

class Button
{
public:
	enum Action {
		NO_ACTION,
		GOTO_MAIN_MENU,
		GOTO_ENTER_NAME_MENU,
		GOTO_GAME,
		GOTO_HELP,
		GOTO_VOLUME,
		VOLUME_UP,
		VOLUME_DOWN,
		TUT_OFF
	};
private:
	Sprite sprite;
	std::string text; // convert to Text
	Action action;
	bool enabled = true;

public:
	Button() {};
	Button(const Sprite& s, const std::string& t, const Action& a);
	void Draw();
	void SetPos(const DirectX::XMFLOAT2& pos);

	void SetColor(const DirectX::SimpleMath::Vector4& color);
	void Activate();
};

class GameTimer;

class FadeText
{
	float mTimer = 1.0f;
public:
	Text mText;
	float mSpeed = 1.0f;
	void Activate(const std::string& str, float t);
	void Update(const GameTimer& gt);
	void Draw();
};
// Handles fonts and viewport sprites
class SpriteManager
{
	std::unique_ptr<DirectX::GraphicsMemory> mGraphicsMemory;
	std::unique_ptr<DirectX::SpriteBatch> mSpriteBatch;
	std::array<std::unique_ptr<DirectX::SpriteFont>, FONT_SIZE> mSpriteFont;

public:

	SpriteManager() {};
	void Init(ID3D12Device* device, ID3D12CommandQueue* commandQueue, UINT srvDescSize, DXGI_FORMAT backBufferFormat, DXGI_FORMAT depthStencilFormat, CD3DX12_CPU_DESCRIPTOR_HANDLE cpuHandle, CD3DX12_GPU_DESCRIPTOR_HANDLE gpuHandle);
	void CreateFontResource(ID3D12Device* device, ID3D12CommandQueue* commandQueue, UINT srvDescSize, DXGI_FORMAT backBufferFormat, DXGI_FORMAT depthStencilFormat);
	void OnResize(D3D12_VIEWPORT& viewport);
	// Called in GameApp::Draw method before drawing sprites and fonts
	void DrawBegin(ID3D12GraphicsCommandList* commandList, const D3D12_VIEWPORT& veiwport);
	// Called in GameApp::Draw method after drawing sprites and fonts
	void DrawEnd();
	// Called in State::Draw method. Begin and End methods are called in GameApp::Draw
	void DrawSprite(const std::string& textureName);
	// Called in State::Draw method. Begin and End methods are called in GameApp::Draw
	void DrawSprite(const Sprite& sprite);
	// Called in State::Draw method. Begin and End methods are called in GameApp::Draw
	void DrawString(size_t i, const std::string& output, const DirectX::XMFLOAT2& pos, bool centre = false);

	void DrawString(const Text& t);
};