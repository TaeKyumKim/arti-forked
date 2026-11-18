"""읽을 문장 준비: 안내 문구(guide_lines.yaml)와 작품 정보(지금 artworks_sample.yaml, 나중에 DB)."""

import yaml


def load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


class GuideLines:
    """guide_lines.yaml: TTS 설정(목소리·빠르기), 안내 키 → 문장."""

    def __init__(self, path):
        data = load_yaml(path)
        tts = data.get('tts') or {}
        self.voice = tts.get('voice', 'ko-KR-SunHiNeural')
        self.rate = tts.get('rate', '+0%')
        self.guides = {str(k): str(v).strip() for k, v in (data.get('guides') or {}).items()}


class YamlArtworkSource:
    """작품 정보 출처 (지금은 artworks_sample.yaml). DB가 정해지면 get·all_artworks가 같은 클래스로 교체."""

    def __init__(self, path):
        rows = load_yaml(path).get('artworks') or []
        self._artworks = {str(r['artwork_id']): r for r in rows if r.get('artwork_id')}

    def get(self, artwork_id):
        return self._artworks.get(artwork_id)

    def all_artworks(self):
        return dict(self._artworks)


def artwork_text(artwork):
    """작품 정보에서 읽을 문장 = description 그대로 (제목·작가는 읽지 않는다)."""
    text = str(artwork.get('description') or '').strip()
    if not text:
        raise ValueError('작품 description이 비어 있음')
    return text
