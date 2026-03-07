import React, { useState } from "react";

export default function ForgotPassword() {
    const [email, setEmail] = useState("");

    async function handleSubmit(e) {
    }
    return (
            <div className="min-h-screen bg-white text-gray-900 flex flex-col px-6 py-12 lg:px-8 mt-[100px]">
                <div className="sm:mx-auto sm:w-full sm:max-w-sm">
                    <h1 className="mt-10 text-center text-2xl font-bold">
                        Forgot Password
                    </h1>
                </div>
                <div className="mt-10 sm:mx-auto sm:w-full sm:max-w-sm">
                    <form className="space-y-6" onSubmit={handleSubmit}>
                        <div>
                            <label htmlFor="email" className="block text-sm font-medium text-gray-900">
                                Email address
                            </label>
                            <div className="mt-2">
                                <input
                                    id="email"
                                    name="email"
                                    type="email"
                                    autoComplete="email"
                                    onChange={(e) => setEmail(e.target.value)}
                                    value={email}
                                    required
                                    className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                                    placeholder="Email address"
                                />
                            </div>
                        </div>
                    </form>
                </div>
            </div>

    );
}