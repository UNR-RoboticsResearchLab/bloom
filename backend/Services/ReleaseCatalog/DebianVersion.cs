// bloom
// DebianVersion.cs
// The version strings OTA pushes are Debian versions of bloom-robot-core (docs/ota.md), so they are
// validated and ordered the way the robot's dpkg will treat them, not as SemVer: "1.0.20~gabc-1"
// sorts before "1.0.20-1".

using System.Text.RegularExpressions;

namespace bloom.Services.ReleaseCatalog
{
    public static class DebianVersion
    {
        // Same pattern the agent applies to a manifest's version (ota_manifest.cpp): no epoch.
        private static readonly Regex Valid = new(@"^[0-9][A-Za-z0-9.+~-]{0,63}$", RegexOptions.Compiled);

        public static bool IsValid(string? version) => version != null && Valid.IsMatch(version);

        // dpkg --compare-versions, without epochs.
        public static int Compare(string a, string b)
        {
            var (upA, revA) = Split(a);
            var (upB, revB) = Split(b);
            var c = CompareFragment(upA, upB);
            return c != 0 ? c : CompareFragment(revA, revB);
        }

        private static (string Upstream, string Revision) Split(string v)
        {
            var dash = v.LastIndexOf('-');
            return dash < 0 ? (v, "") : (v[..dash], v[(dash + 1)..]);
        }

        // ~ sorts before everything, even the end of the string; letters before other symbols.
        private static int Order(char c) =>
            char.IsAsciiDigit(c) ? 0 : char.IsAsciiLetter(c) ? c : c == '~' ? -1 : c + 256;

        // dpkg's verrevcmp: alternate non-digit runs (compared by Order) and numeric runs.
        private static int CompareFragment(string a, string b)
        {
            int i = 0, j = 0;
            while (i < a.Length || j < b.Length)
            {
                while ((i < a.Length && !char.IsAsciiDigit(a[i])) || (j < b.Length && !char.IsAsciiDigit(b[j])))
                {
                    var ac = i < a.Length && !char.IsAsciiDigit(a[i]) ? Order(a[i]) : 0;
                    var bc = j < b.Length && !char.IsAsciiDigit(b[j]) ? Order(b[j]) : 0;
                    if (ac != bc) return ac - bc;
                    if (i < a.Length && !char.IsAsciiDigit(a[i])) i++;
                    if (j < b.Length && !char.IsAsciiDigit(b[j])) j++;
                }
                while (i < a.Length && a[i] == '0') i++;
                while (j < b.Length && b[j] == '0') j++;
                var firstDiff = 0;
                while (i < a.Length && char.IsAsciiDigit(a[i]) && j < b.Length && char.IsAsciiDigit(b[j]))
                {
                    if (firstDiff == 0) firstDiff = a[i] - b[j];
                    i++;
                    j++;
                }
                if (i < a.Length && char.IsAsciiDigit(a[i])) return 1;
                if (j < b.Length && char.IsAsciiDigit(b[j])) return -1;
                if (firstDiff != 0) return firstDiff;
            }
            return 0;
        }
    }
}
