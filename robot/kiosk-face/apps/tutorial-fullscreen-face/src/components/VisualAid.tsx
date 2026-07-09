import type { VisualAidState } from "../hooks/useFaceBridge";
import { resolveVisualAidImageUrl } from "../hooks/useFaceBridge";

// Mirrors face_node.py's draw_visual_aid: a single image gets a full-bleed
// header/footer layout, two images split the screen 50/50 with a divider
// (used for homophone-pair comparisons, e.g. "sea" vs "see").
export function VisualAid({ aid }: { aid: VisualAidState }) {
  if (!aid || aid.images.length === 0) return null;

  return (
    <div className="visual-aid-overlay">
      {aid.images.map((filename, index) => (
        <div className="visual-aid-panel" key={`${filename}-${index}`}>
          {aid.labels[index] && <div className="visual-aid-label">{aid.labels[index]}</div>}
          <img className="visual-aid-image" src={resolveVisualAidImageUrl(filename)} alt={aid.labels[index] ?? ""} />
          {aid.footers[index] && <div className="visual-aid-footer">{aid.footers[index]}</div>}
        </div>
      ))}
    </div>
  );
}
