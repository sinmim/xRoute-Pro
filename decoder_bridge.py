import http.server
import socketserver
import json
import subprocess
import re
import os

# --- CONFIGURATION ---
PIO_ENVIRONMENT_NAME = "esp32doit-devkit-v1" 
PORT = 9999
COMMAND_TIMEOUT = 90
RESULT_FILENAME = "latest_decoded_output.txt"
# --- END CONFIGURATION ---


class MyHttpRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/get-result':
            try:
                with open(RESULT_FILENAME, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                response = {'decoded_text': content}
                self.wfile.write(json.dumps(response).encode('utf-8'))
            except FileNotFoundError:
                self.send_error(404, f"Result file '{RESULT_FILENAME}' not found. Please run a decode first.")
            except Exception as e:
                self.send_error(500, f"Error reading result file: {e}")
        else:
            # Fallback to serving files from the current directory if needed
            super().do_GET()

    def do_POST(self):
        if self.path == '/decode':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data)
            
            report_text = data.get('report', '')
            addresses = re.findall(r'0x[0-9A-Fa-f]{8,10}', report_text)
            
            if not addresses:
                # Send a quick error response
                self.send_response(400)
                self.send_header("Content-type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({'message': 'No addresses found in report'}).encode('utf-8'))
                return

            # Immediately send a success response to the browser to prevent timeout
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            response = {'message': 'Decode process started. Fetch result shortly.'}
            self.wfile.write(json.dumps(response).encode('utf-8'))

            # Now, perform the long-running task *after* responding to the browser
            self.run_decode_process(addresses)
            return

    def run_decode_process(self, addresses):
        print("\n----------------------------------------")
        print(f"Starting decode process for {len(addresses)} addresses in the background...")
        command = ['pio', 'run', '--target', 'decode', '--environment', PIO_ENVIRONMENT_NAME]
        
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=os.getcwd()
            )
            input_str = "\n".join(addresses)
            stdout, stderr = process.communicate(input=input_str, timeout=COMMAND_TIMEOUT)
            
            if process.returncode != 0:
                decoded_text = f"ERROR: PlatformIO decoder failed.\n\n--- STDERR ---\n{stderr}"
            else:
                decoded_lines = [line for line in stdout.splitlines() if line.strip().startswith('0x')]
                if not decoded_lines:
                    decoded_text = f"Decoder ran successfully, but no matching lines found.\nThis can happen if addresses are not in the firmware.\n\n--- RAW STDOUT ---\n{stdout}"
                else:
                    decoded_text = "\n".join(decoded_lines)
            
            print("\n--- DECODED OUTPUT ---")
            print(decoded_text)
            print("----------------------------------------")
            
            # Save the result to a file
            with open(RESULT_FILENAME, 'w', encoding='utf-8') as f:
                f.write(decoded_text)
            print(f"Result saved to '{RESULT_FILENAME}'")

        except Exception as e:
            print(f"\n--- AN ERROR OCCURRED DURING DECODING ---\n{e}")
            with open(RESULT_FILENAME, 'w', encoding='utf-8') as f:
                f.write(f"An error occurred during the decode process:\n{e}")

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With, Content-type")
        self.end_headers()

# --- Main execution ---
Handler = MyHttpRequestHandler
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("============================================")
    print(f"Final Decoder Bridge Serving at http://localhost:{PORT}")
    print(f"Project environment: '{PIO_ENVIRONMENT_NAME}'")
    print(f"Results will be saved to '{RESULT_FILENAME}'")
    print("============================================")
    httpd.serve_forever()