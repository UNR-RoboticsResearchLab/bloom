


export default function SignUp() {

    return (
        // <div className="min-h-screen bg-white text-gray-900 flex flex-col justify-center px-6 py-12 lg:px-8">
        <div className="min-h-screen bg-white text-gray-900 flex flex-col px-6 py-12 lg:px-8 mt-[100px]">
            <div className="sm:mx-auto sm:w-full sm:max-w-sm">
                <h2 className="mt-10 text-center text-2xl font-bold tracking-tight">
                    Sign Up for an account
                </h2>
            </div>

            <div className="mt-10 sm:mx-auto sm:w-full sm:max-w-sm">
                <form className="space-y-6">
                    {/* Email */}
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
                                required
                                className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                                placeholder="name@mail.com"
                            />
                        </div>
                    </div>

                    {/* Password */}
                    <div>
                        <div className="flex items-center justify-between">
                            <label htmlFor="password" className="block text-sm font-medium text-gray-900">
                                Password
                            </label>
                        </div>
                        <div className="mt-2 relative">
                            <input
                                id="password"
                                name="password"
                                type="password"
                                autoComplete="current-password"
                                required
                                className="block w-full rounded-md bg-white px-3 py-1.5 pr-10 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                                placeholder="********"
                            />
                        </div>
                    </div>
                    
                    {/*Full Name */}
                    <div>
                        <div className="flex items-center justify-between">
                            <label htmlFor="fullName" className="block text-sm font-medium text-gray-900">
                                Full Name
                            </label>
                        </div>
                        <div className="mt-2">
                            <input
                                id="fullName"
                                name="fullName"
                                type="text"
                                autoComplete="name"
                                required
                                className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                                placeholder="First M. Last"
                            />
                        </div>
                    </div>

                    {/* Role Selection */}
                    <div>
                        <label htmlFor="role" className="block text-sm font-medium text-gray-900">
                        Role
                        </label>
                        <div className="mt-2">
                            <select
                                id="role"
                                name="role"
                                required
                                className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                                >
                                <option value="">Select a role</option>
                                <option value="teacher">Teacher</option>
                                <option value="student">Student</option>
                                <option value="slp">SLP</option>
                                <option value="admin">Admin</option>
                            </select>
                        </div>
                    </div>
                    
                    {/* Submit Button */}
                    <div>
                        <button
                            type="submit"
                            className="flex w-full justify-center rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
                >
                            Sign Up
                        </button>
                    </div>
                </form>
            </div>
        </div>
    );


}