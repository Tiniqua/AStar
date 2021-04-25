#include "SpriteManager.h"
#include "ResourceUploadBatch.h"
#include "DescriptorHeap.h"

#include "Common/d3dUtil.h"
#include "Common/d3dx12.h"
#include "GameApp.h"
#include "DeltaTimer.h"


#define IID_PPV_ARGS(ppType) __uuidof(**(ppType)), IID_PPV_ARGS_Helper(ppType)



#ifndef ThrowIfFailed
#define ThrowIfFailed(x)                                              \
{                                                                     \
    HRESULT hr__ = (x);                                               \
    std::wstring wfn = AnsiToWString(__FILE__);                       \
    if(FAILED(hr__)) { throw DxException(hr__, L#x, wfn, __LINE__); } \
}
#endif

void SpriteManager::Init(ID3D12Device * device, ID3D12CommandQueue* commandQueue, UINT srvDescSize, DXGI_FORMAT backBufferFormat, DXGI_FORMAT depthStencilFormat, CD3DX12_CPU_DESCRIPTOR_HANDLE cpuHandle, CD3DX12_GPU_DESCRIPTOR_HANDLE gpuHandle)
{

	UINT size = GameApp::Get().GetCbvSrvDescriptorSize();

	mGraphicsMemory = std::make_unique<DirectX::GraphicsMemory>(device);

	DirectX::ResourceUploadBatch resourceUpload(device);

	resourceUpload.Begin();
	{
		DirectX::RenderTargetState rtState(backBufferFormat, depthStencilFormat);
		DirectX::SpriteBatchPipelineStateDescription sbPipelineDesc(rtState, &CommonStates::NonPremultiplied, nullptr, nullptr);

		mSpriteBatch = std::make_unique<DirectX::SpriteBatch>(device, resourceUpload, sbPipelineDesc);

		mSpriteFont.at(0) = std::make_unique<DirectX::SpriteFont>(
			device,
			resourceUpload,
			L"Data/Fonts/Subway_Novella_16.spritefont",
			cpuHandle,
			gpuHandle
			);

		cpuHandle.Offset(size);
		gpuHandle.Offset(size);

		//mSpriteFont.at(1) = std::make_unique<DirectX::SpriteFont>( .. )
		mSpriteFont.at(1) = std::make_unique<DirectX::SpriteFont>(
			device,
			resourceUpload,
			L"Data/Fonts/Traveling__Typewriter_16.spritefont",
			cpuHandle,
			gpuHandle
			);
	}

	auto uploadResourceEnd = resourceUpload.End(commandQueue);
	uploadResourceEnd.wait();


}

void SpriteManager::CreateFontResource(ID3D12Device * device, ID3D12CommandQueue * commandQueue, UINT srvDescSize, DXGI_FORMAT backBufferFormat, DXGI_FORMAT depthStencilFormat)
{
}

void SpriteManager::OnResize(D3D12_VIEWPORT & viewport)
{
}

void SpriteManager::DrawBegin(ID3D12GraphicsCommandList * commandList, const D3D12_VIEWPORT & viewport)
{
	mSpriteBatch->SetViewport(viewport);
	mSpriteBatch->Begin(commandList);
}

void SpriteManager::DrawEnd()
{
	// Prints debug string
	mSpriteFont.at(1)->DrawString(mSpriteBatch.get(), GameApp::Get().mDebugLog.str().c_str(), DirectX::XMFLOAT2(10.0f, 10.0f));

	//Ends spritebatch
	mSpriteBatch->End();

	// clears stream
	GameApp::Get().mDebugLog.str(std::string());
}

void SpriteManager::DrawSprite(const std::string & textureName)
{
	mSpriteBatch->Draw(GameApp::Get().GetSpriteGpuDescHandle(textureName), XMUINT2(500, 50), XMFLOAT2(500.0f, 50.0f), { 1.0f,1.0f,1.0f,1.0f });
}

void SpriteManager::DrawSprite(const Sprite & s)
{
	if (s.destinationRectangle.left < 0)
	{
		mSpriteBatch->Draw(
			s.texture,
			s.textureSize,
			s.position,
			&s.sourceRectangle,
			s.color,
			s.rotation,
			s.origin,
			s.scale,
			s.effects
		);
	}
	else
	{
		mSpriteBatch->Draw(
			s.texture,
			s.textureSize,
			s.destinationRectangle,
			&s.sourceRectangle,
			s.color,
			s.rotation,
			s.origin,
			s.effects
		);
	}


}

void SpriteManager::DrawString(size_t i, const std::string & output, const DirectX::XMFLOAT2& pos, bool centre)
{
	if (i < mSpriteFont.size())
	{
		XMFLOAT2 position;
		if (centre)
		{
			// gets centre point in string
			XMStoreFloat2(&position, mSpriteFont.at(i)->MeasureString(output.c_str(), true) *-0.5f);
		}
		// Applies position
		position.x += pos.x;
		position.y += pos.y;

		mSpriteFont.at(i)->DrawString(mSpriteBatch.get(), output.c_str(), position);
	}
	else
	{
		// i out of range
		assert(false);
	}

}

void SpriteManager::DrawString(const Text & t)
{
	if (t.fontIndex < mSpriteFont.size())
	{
		const char* string = t.string.c_str();

		const DirectX::SimpleMath::Vector2 origin = (t.center) ?
			mSpriteFont.at(t.fontIndex)->MeasureString(string, true) *0.5f : //center 
			t.origin;															// predefined origin

		mSpriteFont.at(t.fontIndex)->DrawString(mSpriteBatch.get(), string, t.position, t.color, t.rotation, origin, t.scale);
	}
	else
	{
		assert(false);
	}
}

Sprite::Sprite(const std::string & textureName)
{
	Initialise(textureName);
}

void Sprite::Initialise(const std::string & textureName, bool centreOrigin)
{
	texture = GameApp::Get().GetSpriteGpuDescHandle(textureName);
	if (centreOrigin)
	{
		origin.x = 0.5f * (float)(sourceRectangle.right - sourceRectangle.left);
		origin.y = 0.5f * (float)(sourceRectangle.bottom - sourceRectangle.top);
	}
}

void Sprite::Draw()
{
	GameApp::Get().DrawSprite(*this);
}

Button::Button(const Sprite & s, const std::string & t, const Action & a)
	:
	sprite(s),
	text(t),
	action(a)
{

}

void Button::Draw()
{
	sprite.Draw();

	GameApp::Get().DrawString(1, text, sprite.position, true);
}

void Button::SetPos(const XMFLOAT2 & pos)
{
	sprite.position = pos;
}

void Button::SetColor(const DirectX::SimpleMath::Vector4 & color)
{
	sprite.color = color;
}

void Button::Activate()
{
	switch (action)
	{
	case Button::NO_ACTION:
		break;
	case Button::GOTO_MAIN_MENU:
		GameApp::Get().ChangeState(GC::STATE_MAINMENU);
		break;
	case Button::GOTO_HELP:
	  GameApp::Get().ChangeState(GC::STATE_HELP);
	  break;
	case Button::GOTO_ENTER_NAME_MENU:
		GameApp::Get().ChangeState("EnterName");
		break;
	case Button::GOTO_GAME:
		GameApp::Get().ChangeState(GC::STATE_PLAY);
		break;
	case Button::GOTO_VOLUME:
		GameApp::Get().ChangeState("Volume"); // todo Luc add to constants as STATE_VOLUME
		break;
	case Button::VOLUME_UP:
		GameApp::Get().GetAudio().SetVolume(0.1f, true); // todo Luc add to constants
		break;
	case Button::VOLUME_DOWN:
		GameApp::Get().GetAudio().SetVolume(-0.1f, true); // todo Luc add to constants
		break;
	case Button::TUT_OFF: // TODO: IMPLEMENT TUT OFF HERE
		GameApp::Get().menusShown = GC::NO_TUTORIAL_VALUE;
		break;
	default:
		break;
	}
}

void Text::Draw()
{
	GameApp::Get().DrawString(*this);
}

void Panel::CalcSpriteRects()
{
	long midX = (mSourceRect.right + mSourceRect.left) >> 1;
	long midY = (mSourceRect.bottom + mSourceRect.top) >> 1;

	//subdivided by 1
	long sizeX = (mSourceRect.right - mSourceRect.left) >> 1;
	long sizeY = (mSourceRect.bottom - mSourceRect.top) >> 1;

	// top left
	const std::array<RECT, COUNT> slice =
	{
		RECT{mSourceRect.left,		mSourceRect.top,			midX,				midY	},				//CORNER_TL
		RECT{midX,					mSourceRect.top,			mSourceRect.right ,	midY	},				//CORNER_TR
		RECT{mSourceRect.left,		midY,					midX,				mSourceRect.bottom},		//CORNER_BL
		RECT{midX,					midY,					mSourceRect.right,	mSourceRect.bottom},		//CORNER_BR
		//edges
		RECT{mSourceRect.left,					midY,					midX,		midY + 1},				// left edge
		RECT{midX,					mSourceRect.top,					midX + 1,		midY},				// top edge
		RECT{midX,					midY,					mSourceRect.right,		midY + 1},				// right edge
		RECT{midX,					midY,					midX + 1,					mSourceRect.bottom},// top edge

		RECT{midX,					midY,					midX + 1,					midY + 1},			// middle
	};

	long inOffsetX = midX >> 1;
	long inOffestY = midY >> 1;

	const std::array<RECT, COUNT> destRects =
	{
		RECT{	mDestRect.left,mDestRect.top,	mDestRect.left + sizeX,	mDestRect.top + sizeY	},//CORNER_TL

		RECT{	mDestRect.right - sizeX,  mDestRect.top,mDestRect.right,mDestRect.top + sizeY},//CORNER_TR
		RECT{	mDestRect.left, mDestRect.bottom - sizeY, mDestRect.left + sizeX, mDestRect.bottom},//CORNER_BL
		RECT{	mDestRect.right - sizeX,mDestRect.bottom - sizeY,mDestRect.right,mDestRect.bottom},//CORNER_BR

		RECT{	mDestRect.left,mDestRect.top + sizeY,mDestRect.left + sizeX,mDestRect.bottom - sizeY}, //EDGE_L
		RECT{	mDestRect.left + sizeX,mDestRect.top,mDestRect.right - sizeX,mDestRect.top + sizeY}, //edge t
		RECT{	mDestRect.right - sizeX,	mDestRect.top + sizeY,	mDestRect.right,	mDestRect.bottom - sizeY	},
		RECT{	mDestRect.left + sizeX,	mDestRect.bottom - sizeY,	mDestRect.right - sizeX,	mDestRect.bottom	},

		RECT{mDestRect.left + sizeX,	mDestRect.top + sizeY,	mDestRect.right - sizeX,	mDestRect.bottom - sizeY	}
	};



	for (size_t i = 0; i < COUNT; ++i)
	{
		mSprites.at(i).sourceRectangle = slice.at(i);
		mSprites.at(i).destinationRectangle = destRects.at(i);
	}
}

void Panel::Initialize(const std::string & textureName, const RECT & src, const RECT & dst)
{
	mSourceRect = src;
	mDestRect = dst;

	for (auto& s : mSprites)
	{
		s.Initialise(textureName);
	}

	CalcSpriteRects();

}

void Panel::Move(DirectX::SimpleMath::Vector2 v, bool increment)
{
	if (increment)
	{
		mDestRect.top += (long)v.y;
		mDestRect.bottom += (long)v.y;
		mDestRect.right += (long)v.x;
		mDestRect.left += (long)v.x;
	}
	else
	{
		const long w = mDestRect.right - mDestRect.left;
		const long h = mDestRect.bottom - mDestRect.top;

		mDestRect.top = (long)v.y;
		mDestRect.bottom = mDestRect.top + h;

		mDestRect.left = (long)v.x;
		mDestRect.right = mDestRect.left + w;
	}

	CalcSpriteRects();
}

const SimpleMath::Vector2 Panel::GetPos() const
{
	return { (float)mDestRect.left	,(float)mDestRect.top };
}

void Panel::Draw()
{
	for (auto& s : mSprites)
	{
		s.Draw();
	}
}

void FadeText::Activate(const std::string & str, float t)
{
	mText.string = str;
	mTimer = t;
}

void FadeText::Update(const GameTimer & gt)
{
	if (mTimer >= 0.0f)
	{
		mTimer -= gt.DeltaTime() * mSpeed;
	}
	else
	{
		mTimer = 0.0f;
	}

	mText.color.w = mTimer;
}

void FadeText::Draw()
{
	if (mTimer > 0.0f)
	{
		mText.Draw();
	}
}