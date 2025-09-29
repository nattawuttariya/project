import os
from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from pymongo import MongoClient
from datetime import datetime, timezone, timedelta
import json
from functools import wraps
import logging
from dotenv import load_dotenv

import requests

# Redis for rate limiting
import redis
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address

# โหลดตัวแปรจาก .env
load_dotenv()

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ตั้งค่าการ log
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s: %(message)s',
    handlers=[
        logging.FileHandler("app.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# โหลดตัวแปรสำคัญจาก .env
API_KEY = os.environ.get("API_KEY")
ESP32_URL = os.environ.get("ESP32_URL")
MONGO_URI = os.environ.get("MONGO_URI")
REDIS_URL = os.environ.get("REDIS_URL")

# ตรวจสอบการตั้งค่า
if not all([API_KEY, ESP32_URL, MONGO_URI]):
    logger.error("API_KEY, ESP32_URL, หรือ MONGO_URI ไม่ถูกตั้งค่าใน .env")
    raise RuntimeError("API_KEY, ESP32_URL, หรือ MONGO_URI ไม่ถูกตั้งค่าใน .env")

# เชื่อม Redis ถ้ามี
redis_client = None
if REDIS_URL:
    try:
        redis_client = redis.StrictRedis.from_url(REDIS_URL)
        redis_client.ping()
        logger.info("Connected to Redis successfully.")
    except Exception as e:
        logger.warning(f"Cannot connect to Redis: {e}")
        redis_client = None

# ตั้งค่า Rate Limiter
try:
    if redis_client:
        limiter = Limiter(
            key_func=get_remote_address,
            storage_uri=REDIS_URL,
            default_limits=["100 per hour"]
        )
        logger.info("Using Redis for rate limiting.")
    else:
        raise Exception("Redis client not available")
except Exception as e:
    logger.warning(f"Redis not available or error occurred, fallback to in-memory rate limiting: {e}")
    limiter = Limiter(
        key_func=get_remote_address,
        storage_uri="memory://",
        default_limits=["100 per hour"]
    )
limiter.init_app(app)

# MongoDB Setup
client = MongoClient(MONGO_URI)
db = client['Data1']
customers_collection = db['customers']
cash_collection = db["cash"]
points_collection = db['points']
scan_collection = db["scan"]


def require_api_key(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        api_key = request.headers.get('X-Api-Key') or request.headers.get('x-api-key')
        if not api_key and request.is_json:
            api_key = request.json.get('api_key')
        if api_key != API_KEY:
            return jsonify({"message": "Invalid API Key."}), 403
        return f(*args, **kwargs)
    return decorated_function


def is_valid_user_id(user_id):
    return isinstance(user_id, str) and 1 <= len(user_id) <= 32

def is_valid_phone(user_id):
    return user_id.isdigit() and len(user_id) == 10

@app.route('/save-transaction', methods=['POST'])
@limiter.limit("10 per minute")
@require_api_key
def save_transaction():
    try:
        logger.info("====== NEW /save-transaction REQUEST ======")
        logger.info(f"Headers: {dict(request.headers)}")
        logger.info(f"Raw Data: {request.data.decode('utf-8', errors='ignore')}")

        if not request.is_json:
            return jsonify({"message": "Invalid or missing JSON in request."}), 400

        data = request.get_json()

        user_id = data.get("user_id")
        amount_raw = data.get("amount")
        points_used = int(data.get("points") or 0)
        payment_method = data.get("paymentMethod", "").lower()

        credit = data.get("credit", 0)
        try:
            credit = int(credit)
        except:
            credit = 0

        timestamp = datetime.now(timezone.utc) + timedelta(hours=7)
        timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S')

        if not user_id:
            try:
                cash_collection.insert_one({
                    "data": data,
                    "timestamp": timestamp_str
                })
                logger.info("Saved data to cash_collection due to missing user_id")
            except Exception as e:
                logger.error(f"Error saving to cash_collection (missing user_id): {e}")
            return jsonify({"message": "Missing user_id. Data saved."}), 400

        try:
            amount = float(amount_raw) if amount_raw not in [None, ''] else None
        except (ValueError, TypeError):
            return jsonify({"message": "Invalid amount format."}), 400

        # แก้ไขตรงนี้: ตอนนี้จะต้องกรอกอย่างน้อย amount, points, หรือ credit
        if amount is None and points_used == 0 and credit == 0:
            return jsonify({"message": "Either amount, points, or credit must be provided."}), 400

        # บันทึก transaction แบบ cash
        if ((user_id.isdigit() and len(user_id) == 10) or len(user_id) > 11) and payment_method == "cash":
            temp_last_amount = round((amount or 0) - (points_used / 10) - credit, 2)
            cash_doc = {
                "user_id": user_id,
                "amount": amount_raw,
                "last_amount": temp_last_amount,
                "paymentMethod": payment_method,
                "timestamp": timestamp_str
            }
            if credit > 0:
                cash_doc["credit"] = credit
            try:
                cash_collection.insert_one(cash_doc)
                logger.info(f"Saved cash transaction to cash_collection (user_id: {user_id})")
            except Exception as e:
                logger.error(f"Error saving cash transaction for user_id {user_id}: {e}")

        customer = customers_collection.find_one({"user_id": user_id}) if is_valid_phone(user_id) else None
        current_points = customer.get("points", 10) if customer else 10
        old_credit = customer.get("credit", 0) if customer else 0

        if customer is None and is_valid_phone(user_id):
            try:
                customers_collection.insert_one({
                    "user_id": user_id,
                    "points": current_points,
                    "credit": credit,
                    "timestamp": timestamp_str
                })
                logger.info("Inserted new customer (valid phone number)")
            except Exception as e:
                logger.error(f"Error inserting new customer: {e}")
                return jsonify({"message": "Failed to create new customer."}), 500

        if is_valid_phone(user_id):
            if points_used > current_points:
                return jsonify({"message": "Insufficient points."}), 400

            discount = points_used / 10
            earned_points = int(amount) if amount else 0
            total_points = current_points + earned_points - points_used

            new_credit = max(old_credit - credit, 0)

            last_amount = round((amount or 0) - discount - credit, 2) if (amount or credit) else None

            try:
                customers_collection.update_one(
                    {"user_id": user_id},
                    {"$set": {
                        "points": total_points,
                        "credit": new_credit,
                        "timestamp": timestamp_str
                    }},
                    upsert=True
                )
                logger.info("Updated customer points and credit (valid phone)")
            except Exception as e:
                logger.error(f"Error updating customer points and credit: {e}")
                return jsonify({"message": "Failed to update points and credit."}), 500
        else:
            if points_used > 0:
                return jsonify({"message": "Points cannot be used with this user_id."}), 400
            discount = 0
            earned_points = 0
            total_points = 0
            last_amount = round((amount or 0) - credit, 2) if (amount or credit) else None

        transaction_data = {
            "user_id": user_id,
            "amount": f"{amount:.2f} บาท" if amount is not None else None,
            "points_used": points_used,
            "earned_points": earned_points,
            "total_points": total_points,
            "last_amount": f"{last_amount:.2f} บาท" if last_amount is not None else None,
            "paymentMethod": payment_method,
            "timestamp": timestamp_str
        }

        if is_valid_phone(user_id):
            try:
                points_collection.insert_one({
                    "user_id": user_id,
                    "timestamp": timestamp_str,
                    "earned_points": earned_points,
                    "points_used": points_used,
                    "total_points": total_points,
                    "amount": amount,
                    "last_amount": last_amount,
                    "discount": discount,
                    "credit": credit,
                    "finally_amount": round((amount or 0) + credit, 2) if (amount or credit) else None,
                    "paymentMethod": payment_method,
                    "activityLog": [
                        {
                            "type": "Earned",
                            "points": earned_points,
                            "timestamp": timestamp_str,
                            "note": f"Earned {earned_points} points from {amount:.2f} บาท" if earned_points else None
                        },
                        {
                            "type": "Redeemed",
                            "points": points_used,
                            "timestamp": timestamp_str,
                            "note": f"Redeemed {points_used} points (discount {discount:.2f} บาท)" if points_used else None
                        }
                    ]
                })
                logger.info("Saved points transaction")
            except Exception as e:
                logger.error(f"Error saving points transaction: {e}")
                return jsonify({"message": "Failed to save points transaction."}), 500
        else:
            logger.info("Skipped saving to points_collection because user_id is not a phone number")

        socketio.emit("update_points", transaction_data)

        response_data = {
            "message": "Transaction saved successfully",
            "paymentMethod": payment_method,
            "amount": f"{amount:.2f} บาท" if amount is not None else None,
            "last_amount": f"{last_amount:.2f} บาท" if last_amount is not None else None,
            "total_points": total_points,
            "activityLog": [{
                "type": "earn" if earned_points > 0 else "use",
                "points": earned_points if earned_points > 0 else points_used,
                "timestamp": timestamp_str,
                "note": "Auto update via transaction"
            }]
        }

        if not is_valid_phone(user_id):
            response_data["user_id"] = user_id

        return jsonify(response_data), 200

    except Exception as e:
        logger.exception(f"Error in /save-transaction: {e}")
        return jsonify({"message": "Internal server error"}), 500

    

@app.route('/save-pos', methods=['POST', 'GET'])
@limiter.limit("20 per minute")
@require_api_key
def save_pos():
    try:
        if request.method == 'POST':
            if not request.is_json:
                return jsonify({"message": "Invalid or missing JSON in request."}), 400

            data = request.get_json()
            user_id = data.get("user_id")
            if not is_valid_user_id(user_id):
                return jsonify({"message": "Invalid or missing user_id."}), 400

            try:
                request_amount = float(data.get("amount"))
            except Exception:
                return jsonify({"message": "Invalid amount."}), 400

            try:
                points_deducted = int(''.join(filter(str.isdigit, str(data.get("points", "0")))))
            except Exception:
                points_deducted = 0

            discount = points_deducted * 0.10
            net_amount = max(request_amount - discount, 0)

            # หา finally_amount ล่าสุดจาก points_collection เพื่อส่งไป ESP32
            points_record = points_collection.find_one({"user_id": user_id}, sort=[("_id", -1)])
            if points_record and "finally_amount" in points_record:
                try:
                    esp32_amount = float(points_record["finally_amount"])
                except (ValueError, TypeError):
                    esp32_amount = net_amount
            else:
                esp32_amount = net_amount

            current_time = datetime.now()

            # บันทึกข้อมูลลง collection 'scan'
            scan_collection.insert_one({
                'user_id': user_id,
                'amount': f"{net_amount:.2f} บาท",
                'timestamp': current_time.strftime('%Y-%m-%d %H:%M:%S'),
                'paymentMethod': "scan"
            })

            # ส่งข้อมูล POST ไป ESP32 ใช้ esp32_amount และ user_id (เบอร์) ที่คำนวณล่าสุด
            esp32_response_data = {}
            try:
                response = requests.post(
                    ESP32_URL,
                    json={
                        "amount": round(esp32_amount, 2),
                        "phone": user_id   # เพิ่มส่งเบอร์โทรไปด้วย
                    },
                    timeout=5
                )
                response.raise_for_status()
                if 'application/json' in response.headers.get('Content-Type', ''):
                    esp32_response_data = response.json()
                else:
                    esp32_response_data = {"message": response.text}

                logger.info(f"ESP32 response for user_id {user_id}: {esp32_response_data}")

            except requests.exceptions.Timeout:
                logger.warning(f"Timeout while sending data to ESP32 for user_id {user_id}")
                esp32_response_data = {"error": "ESP32 request timed out"}

            except requests.exceptions.RequestException as e:
                logger.error(f"Error sending data to ESP32 for user_id {user_id}: {e}")
                esp32_response_data = {"error": str(e)}

            return jsonify({
                "message": "POS transaction saved successfully",
                "amount": f"{net_amount:.2f} บาท",
                "esp32_response": esp32_response_data
            }), 200

        else:
            return jsonify({"message": "POS endpoint is active. Use POST to save data."}), 200

    except Exception as e:
        logger.exception(f"Error in /save-pos: {e}")
        return jsonify({"message": "Internal server error"}), 500


@app.route('/save-credit', methods=['POST', 'GET'])
@limiter.limit("20 per minute")
@require_api_key
def save_credit():
    try:
        if request.method == 'POST':
            if not request.is_json:
                return jsonify({"message": "Invalid or missing JSON in request."}), 400

            data = request.get_json()
            user_id = data.get("user_id")
            credit_to_use = data.get("credit")  # จำนวนเครดิตที่จะใช้

            # ตรวจสอบ user_id
            if not is_valid_user_id(user_id):
                return jsonify({"message": "Invalid or missing user_id."}), 400

            # ตรวจสอบ credit
            try:
                credit_to_use = float(credit_to_use)
            except Exception:
                return jsonify({"message": "Invalid credit."}), 400

            if credit_to_use <= 0:
                return jsonify({"message": "Credit must be greater than 0."}), 400

            # ดึงข้อมูล customer
            customer = customers_collection.find_one({"user_id": user_id})
            if not customer:
                return jsonify({"message": "Customer not found."}), 404

            old_credit = customer.get("credit", 0)
            if credit_to_use > old_credit:
                return jsonify({"message": "Insufficient credit."}), 400

            # คำนวณเครดิตใหม่
            new_credit = old_credit - credit_to_use
            timestamp = datetime.now(timezone.utc) + timedelta(hours=7)
            timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S')

            # อัปเดต customer
            try:
                customers_collection.update_one(
                    {"user_id": user_id},
                    {"$set": {"credit": new_credit, "timestamp": timestamp_str}}
                )
                logger.info(f"Deducted {credit_to_use} credit from user_id {user_id}, new balance: {new_credit}")
            except Exception as e:
                logger.error(f"Error updating customer credit: {e}")
                return jsonify({"message": "Failed to update customer credit."}), 500

            # บันทึกลง points_collection
            try:
                points_collection.insert_one({
                    "user_id": user_id,
                    "timestamp": timestamp_str,
                    "earned_points": 0,
                    "points_used": 0,
                    "total_points": 0,
                    "amount": 0,
                    "last_amount": 0,
                    "discount": 0,
                    "credit": credit_to_use,
                    "finally_amount": round(credit_to_use, 2),
                    "paymentMethod": "credit_use",
                    "activityLog": [
                        {"type": "Redeemed", "points": 0, "timestamp": timestamp_str, "note": f"Used {credit_to_use} credit"},
                    ]
                })
                logger.info(f"Saved credit deduction transaction for user_id {user_id}")
            except Exception as e:
                logger.error(f"Error saving credit deduction transaction: {e}")

            # ส่งค่าไป ESP32
            esp32_response_data = {}
            try:
                response = requests.post(
                    ESP32_URL,
                    json={
                        "finally_amount": round(credit_to_use, 2),
                        "phone": user_id
                    },
                    timeout=5
                )
                response.raise_for_status()
                if 'application/json' in response.headers.get('Content-Type', ''):
                    esp32_response_data = response.json()
                else:
                    esp32_response_data = {"message": response.text}

                logger.info(f"ESP32 response (credit deduction) for user_id {user_id}: {esp32_response_data}")

            except requests.exceptions.Timeout:
                logger.warning(f"Timeout while sending credit data to ESP32 for user_id {user_id}")
                esp32_response_data = {"error": "ESP32 request timed out"}

            except requests.exceptions.RequestException as e:
                logger.error(f"Error sending credit data to ESP32 for user_id {user_id}: {e}")
                esp32_response_data = {"error": str(e)}

            return jsonify({
                "message": "Credit used successfully and sent to ESP32",
                "credit_used": round(credit_to_use, 2),
                "new_credit_balance": new_credit,
                "esp32_response": esp32_response_data
            }), 200

        else:
            return jsonify({"message": "Credit endpoint is active. Use POST to send data."}), 200

    except Exception as e:
        logger.exception(f"Error in /save-credit: {e}")
        return jsonify({"message": "Internal server error"}), 500



@app.route('/getPoints', methods=['GET'])
@limiter.limit("10 per minute")
def getPoints():
    try:
        user_id = request.args.get('user_id')
        if not user_id:
            return jsonify({"message": "Missing user ID."}), 400

        if not is_valid_user_id(user_id):
            return jsonify({"message": "Invalid user ID format."}), 400

        customer = customers_collection.find_one({"user_id": user_id})
        if customer:
            points = customer.get("points", 0)
            return jsonify({"points": points}), 200
        else:
            return jsonify({"message": "You are not a member yet."}), 404

    except Exception as e:
        logger.error(f"Error in /getPoints: {e}", exc_info=True)
        return jsonify({"message": "Error retrieving points."}), 500


@app.route('/getCredit', methods=['GET'])
def get_credit():
    user_id = request.args.get('user_id')
    if not user_id:
        return jsonify({"error": "Missing user_id parameter"}), 400

    customer = customers_collection.find_one({"user_id": user_id})
    if not customer:
        return jsonify({"error": "User not found"}), 404

    credit = customer.get('credit', 0)
    return jsonify({"credit": credit})


@app.route('/add-credit', methods=['POST'])
@require_api_key
def add_credit():
    try:
        data = request.get_json()
        user_id = data.get("user_id")
        credit_to_add = float(data.get("credit", 0))

        if not is_valid_user_id(user_id):
            return jsonify({"message": "Invalid user_id"}), 400

        timestamp_str = datetime.now(timezone.utc).astimezone(timezone(timedelta(hours=7))).strftime('%Y-%m-%d %H:%M:%S')

        customer = customers_collection.find_one({"user_id": user_id})
        if not customer:
            # ถ้าไม่เจอลูกค้าให้สร้างใหม่
            customers_collection.insert_one({
                "user_id": user_id,
                "points": 10,
                "credit": credit_to_add,
                "timestamp": timestamp_str
            })
            logger.info(f"[CREDIT] Created new customer {user_id} with credit {credit_to_add}")
        else:
            new_credit = customer.get("credit", 0) + credit_to_add
            customers_collection.update_one(
                {"user_id": user_id},
                {"$set": {
                    "credit": new_credit,
                    "timestamp": timestamp_str
                }}
            )
            logger.info(f"[CREDIT] Updated credit for user_id {user_id} to {new_credit}")

        return jsonify({
            "message": "Credit added successfully",
            "user_id": user_id,
            "added_credit": credit_to_add
        }), 200

    except Exception as e:
        logger.exception(f"[CREDIT] Error adding credit: {e}")
        return jsonify({"message": "Internal server error"}), 500


if __name__ == '__main__':
    debug_mode = os.environ.get("FLASK_DEBUG", "false").lower() == "true"

    SSL_CERT_PATH = os.environ.get("SSL_CERT_PATH")
    SSL_KEY_PATH = os.environ.get("SSL_KEY_PATH")

    ssl_context = None
    if SSL_CERT_PATH and SSL_KEY_PATH:
        if os.path.exists(SSL_CERT_PATH) and os.path.exists(SSL_KEY_PATH):
            ssl_context = (SSL_CERT_PATH, SSL_KEY_PATH)
        else:
            logger.warning(f"SSL cert or key file not found: {SSL_CERT_PATH}, {SSL_KEY_PATH}")

    if ssl_context:
        logger.info("Starting Flask app with SSL/TLS")
        app.run(debug=debug_mode, host='0.0.0.0', port=3000, ssl_context=ssl_context)
    else:
        logger.info("Starting Flask app without SSL")
        app.run(debug=debug_mode, host='0.0.0.0', port=3000)
