import argparse
import http.server
import ssl
# import cgi
import os
from pathlib import Path

os.chdir(os.path.dirname(__file__))

def get_ssl_context(certfile, keyfile):
    context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
    context.load_cert_chain(certfile, keyfile)
    context.set_ciphers("@SECLEVEL=1:ALL")
    return context

class HTTPHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Serve index.html when accessing the root ('/')
        print(self.path)
        if self.path == "/":
            self.path = "web/index.html"
        return super().do_GET()

#     def do_POST(self):
#         self.send_response(200)

#         self.send_header('Content-type', 'application/json')
#         self.send_header('Access-Control-Allow-Origin', '*')
#         self.end_headers()

#         content_length = int(self.headers['Content-Length'])
#         ctype, pdict = cgi.parse_header(self.headers['Content-Type'])
#         pdict['boundary'] = bytes(pdict['boundary'], 'utf-8')
#         fields = cgi.parse_multipart(self.rfile, pdict)

#         audio_data = fields.get('audio_data')[0]
#         file_wav = 'query.wav'
#         # file_run = '../test.wav'
#         with open(file_wav, 'wb') as audio_file:
#             audio_file.write(audio_data)
#         self.wfile.write(str(content_length).encode(encoding = 'utf_8'))
#         # os.rename(file_wav, file_run)

def main():
    current_dir = Path(__file__).parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5678, help="Port number to start the server")
    parser.add_argument("--certfile", type=str, default="cert.pem", help="Path to the SSL certificate file")
    parser.add_argument("--keyfile", type=str, default="key.pem", help="Path to the SSL key file")
    args = parser.parse_args()

    server_address = ("0.0.0.0", args.port)
    httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)
    context = get_ssl_context(args.certfile, args.keyfile)
    httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

    httpd.serve_forever()

if __name__ == "__main__":
    main()