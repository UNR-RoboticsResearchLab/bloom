using System.Text.Json;
using bloom.Models.dto;

namespace bloom.Services
{
    /// <summary>
    /// Sends session audio to the Python WhisperX microservice and returns scored results.
    /// </summary>
    public class ArSrTranscriptionService
    {
        private readonly HttpClient _http;
        private readonly ILogger<ArSrTranscriptionService> _logger;

        public ArSrTranscriptionService(IHttpClientFactory factory, ILogger<ArSrTranscriptionService> logger)
        {
            _http   = factory.CreateClient("ArSrMicroservice");
            _logger = logger;
        }

        public async Task<ArSrMicroserviceResult> AnalyzeAsync(
            Stream audioStream,
            string audioFileName,
            int ageMonths,
            int percentile,
            string markersJson)
        {
            using var form = new MultipartFormDataContent();
            using var audioContent = new StreamContent(audioStream);

            form.Add(audioContent,             "session_audio", audioFileName);
            form.Add(new StringContent(ageMonths.ToString()),   "age");
            form.Add(new StringContent(percentile.ToString()),  "percentile");
            form.Add(new StringContent(markersJson),            "markers");

            var response = await _http.PostAsync("/api/analyze", form);
            response.EnsureSuccessStatusCode();

            var json = await response.Content.ReadAsStringAsync();
            var result = JsonSerializer.Deserialize<ArSrMicroserviceResult>(json, new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true
            });

            return result ?? throw new InvalidOperationException("Empty response from transcription service");
        }

        public async Task<bool> IsHealthyAsync()
        {
            try
            {
                var response = await _http.GetAsync("/health");
                return response.IsSuccessStatusCode;
            }
            catch
            {
                return false;
            }
        }
    }
}
