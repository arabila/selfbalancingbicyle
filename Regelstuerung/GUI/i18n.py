import json
import os
from typing import Dict

_current_lang = "de"
_cache: Dict[str, Dict[str, str]] = {}


def _load_lang(lang: str) -> Dict[str, str]:
    global _cache
    if lang in _cache:
        return _cache[lang]

    base_dir = os.path.dirname(__file__)
    locales_dir = os.path.join(base_dir, "locales")
    file_path = os.path.join(locales_dir, f"{lang}.arb")
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            # Flache Key->String Map
            _cache[lang] = data
            return data
    except Exception:
        _cache[lang] = {}
        return {}


def set_locale(lang: str) -> None:
    global _current_lang
    _current_lang = lang if lang in {"de", "en"} else "en"
    _load_lang(_current_lang)


def t(key: str, **kwargs) -> str:
    # 1) Sprache
    lang_map = _load_lang(_current_lang)
    value = lang_map.get(key)
    if value is None:
        # 2) Fallback EN
        value = _load_lang("en").get(key, key)
    try:
        return value.format(**kwargs) if kwargs else value
    except Exception:
        # Falls Platzhalter fehlen, gib rohen Wert zurück
        return value


